/*
==============================================================================
MIT License

Copyright (c) 2024 Ethan M Brown

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================
*/

#include <rclcpp/qos.hpp>
#include "network_bridge/subscription_manager.hpp"

SubscriptionManager::SubscriptionManager(
  const rclcpp::Node::SharedPtr & node, const std::string & topic,
  const std::string & subscribe_namespace, int zstd_compression_level,
  bool publish_stale_data)
: node_(node),
  msg_type_(),
  topic_(topic),
  subscribe_namespace_(subscribe_namespace),
  zstd_compression_level_(zstd_compression_level),
  received_msg_(false),
  is_stale_(true),
  publish_stale_data_(publish_stale_data),
  data_()
{
  topic_found_ = true;   // optimistic
  setup_subscription();
}

void SubscriptionManager::setup_subscription()
{
  if (!rclcpp::ok()) {return;} // Querying graph is fragile

  const auto all_topics_and_types = node_->get_topic_names_and_types();

  std::string topic = subscribe_namespace_ + topic_;

  if (all_topics_and_types.find(topic) == all_topics_and_types.end()) {
    if (topic_found_) {
      RCLCPP_WARN(node_->get_logger(), "Topic %s not found", topic.c_str());
    }
    topic_found_ = false;
    return;
  }

  // Get QoS profile from topic
  auto topic_info = node_->get_publishers_info_by_topic(topic);

  if (topic_info.size() == 0) {
    if (topic_found_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "No publishers found for topic %s", topic.c_str());
    }
    topic_found_ = false;
    return;
  }

  topic_found_ = true;
  auto qos_candidate = topic_info[0].qos_profile().get_rmw_qos_profile();

  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qos_candidate));

  qos.durability(qos_candidate.durability);
  qos.reliability(qos_candidate.reliability);

  if (qos_candidate.deadline.sec < 10000) {
    qos.deadline(
      rclcpp::Duration(
        qos_candidate.deadline.sec, qos_candidate.deadline.nsec));
  }

  qos.liveliness(qos_candidate.liveliness);

  if (qos_candidate.liveliness_lease_duration.sec < 10000) {
    qos.liveliness_lease_duration(
      rclcpp::Duration(
        qos_candidate.liveliness_lease_duration.sec,
        qos_candidate.liveliness_lease_duration.nsec));
  }

  msg_type_ = all_topics_and_types.at(topic)[0];

  subscriber = node_->create_generic_subscription(
    topic, msg_type_, qos,
    [this](
      const std::shared_ptr<const rclcpp::SerializedMessage> & serialized_msg) {
      this->callback(serialized_msg);
    });
}

void SubscriptionManager::callback(
  const std::shared_ptr<const rclcpp::SerializedMessage> & serialized_msg)
{
  RCLCPP_DEBUG(
    node_->get_logger(),
    "Received message on topic %s", topic_.c_str());
  received_msg_ = true;
  is_stale_ = false;
  data_.resize(serialized_msg->size());
  auto buff = serialized_msg->get_rcl_serialized_message().buffer;
  std::copy(buff, buff + serialized_msg->size(), data_.begin());
}

void SubscriptionManager::check_subscription()
{
  if (!subscriber) {
    setup_subscription();
  }
}


bool SubscriptionManager::has_data() const
{
  if (!subscriber) {
    return false;
  }
  if (!received_msg_) {
    return false;
  }
  if (is_stale_ && !publish_stale_data_) {
    return false;
  }
  return true;
}


const std::vector<uint8_t> & SubscriptionManager::get_data()
{
  if (!subscriber) {
    setup_subscription();
    RCLCPP_WARN(node_->get_logger(), "Send Timer: Subscriber is not set");
    return data_;
  }

  if (!received_msg_) {
    RCLCPP_WARN(node_->get_logger(), "Send Timer: No message ever received");
    return data_;
  }


  if (is_stale_ && !publish_stale_data_) {
    RCLCPP_WARN(node_->get_logger(), "Send Timer: Stored data is stale");
    data_.clear();
    return data_;
  }

  is_stale_ = true;
  return data_;
}
