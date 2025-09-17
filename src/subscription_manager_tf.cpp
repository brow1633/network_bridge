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
#include <tf2_ros/qos.hpp>
#include "network_bridge/subscription_manager_tf.hpp"

SubscriptionManagerTF::SubscriptionManagerTF(
  const rclcpp::Node::SharedPtr & node, const std::string & topic,
  const std::string & subscribe_namespace, int zstd_compression_level,
  bool publish_stale_data, bool static_tf)
: SubscriptionManager(node, topic, subscribe_namespace, zstd_compression_level, publish_stale_data),
  static_tf_(static_tf)
{
}

SubscriptionManagerTF::~SubscriptionManagerTF() {}


void SubscriptionManagerTF::check_subscription()
{
  if (!tf2_subscriber_) {
    setup_subscription();
  }
}


void SubscriptionManagerTF::create_subscription(
  const std::string & topic,
  const std::string & /*msg_type*/, const rclcpp::QoS & /*qos*/)
{
  if (static_tf_) {
    tf2_ros::StaticListenerQoS static_qos;
    tf2_subscriber_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      topic, static_qos,
      [this](
        const std::shared_ptr<const tf2_msgs::msg::TFMessage> & tfmsg) {
        this->tf2_callback(tfmsg);
      });
    RCLCPP_INFO(
      node_->get_logger(),
      "Created static TF subscriber for topic %s", topic.c_str());
  } else {
    tf2_ros::DynamicListenerQoS dynamic_qos;
    tf2_subscriber_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      topic, dynamic_qos,
      [this](
        const std::shared_ptr<const tf2_msgs::msg::TFMessage> & tfmsg) {
        this->tf2_callback(tfmsg);
      });
    RCLCPP_INFO(
      node_->get_logger(),
      "Created generic TF subscriber for topic %s", topic.c_str());
  }
}

bool SubscriptionManagerTF::is_stale() const
{
  if (static_tf_) {
    return false;
  }
  return SubscriptionManager::is_stale();
}

void SubscriptionManagerTF::set_include_pattern(const std::vector<std::string> & pattern)
{
  for (auto v: pattern) {
    include_pattern.push_back(std::regex(v));
  }
}

void SubscriptionManagerTF::set_exclude_pattern(const std::vector<std::string> & pattern)
{
  for (auto v: pattern) {
    exclude_pattern.push_back(std::regex(v));
  }
}

void SubscriptionManagerTF::tf2_callback(
  const std::shared_ptr<const tf2_msgs::msg::TFMessage> & tfmsg)
{
  bool new_tf = false;
  for (size_t i = 0; i < tfmsg->transforms.size(); i++) {
    const geometry_msgs::msg::TransformStamped t = tfmsg->transforms[i];
    if (!exclude_pattern.empty()) {
      bool matched = false;
      for (auto v : exclude_pattern) {
        std::smatch m;
        if (std::regex_match(t.header.frame_id, m, v)) {
          matched = true;
          break;
        }
        if (std::regex_match(t.child_frame_id, m, v)) {
          matched = true;
          break;
        }
      }
      if (matched) {
        // Ignore this transform, it's in the exclude list
        continue;
      }
    }
    if (!include_pattern.empty()) {
      bool matched = false;
      for (auto v : include_pattern) {
        std::smatch m;
        if (std::regex_match(t.header.frame_id, m, v)) {
          matched = true;
          break;
        }
        if (std::regex_match(t.child_frame_id, m, v)) {
          matched = true;
          break;
        }
      }
      if (!matched) {
        // Ignore this transform, it's not in the matched list
        continue;
      }
    }
    auto id = std::make_pair(t.header.frame_id, t.child_frame_id);
    auto it = tf_id_.find(id);
    if (it == tf_id_.end()) {
      // Unknown TF
      auto id_rev = std::make_pair(t.child_frame_id, t.header.frame_id);
      auto it_rev = tf_id_.find(id_rev);
      if (it_rev != tf_id_.end()) {
        // We detected TF B->A when A->B was already in the tree.
        RCLCPP_INFO(
          node_->get_logger(), "Detected inconsistent TF (%s->%s) on %s. Resetting buffer.",
          t.header.frame_id.c_str(), t.child_frame_id.c_str(), topic_.c_str());
        tf_id_.clear();
        tfs_.transforms.clear();
        tf2_subscriber_.reset();
        return;
      }
      tf_id_[id] = tfs_.transforms.size();
      tfs_.transforms.push_back(t);
      new_tf = true;
    } else {
      // Known TF
      tfs_.transforms[it->second] = t;
    }
  }
  if (static_tf_) {
    for (size_t i = 0; i < tfs_.transforms.size(); i++) {
      tfs_.transforms[i].header.stamp = rclcpp::Time();
    }
  }
  if (new_tf) {
    RCLCPP_INFO(
      node_->get_logger(), "TF %s list contains %lu transforms",
      topic_.c_str(), tfs_.transforms.size());
  }
  std::shared_ptr<rclcpp::SerializedMessage> serialized_msg(new rclcpp::SerializedMessage);
  tf2_serialization_.serialize_message(&tfs_, serialized_msg.get());
  callback(serialized_msg);
}
