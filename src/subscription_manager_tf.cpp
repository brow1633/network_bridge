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


void SubscriptionManagerTF::create_subscription(
  const std::string & topic,
  const std::string & /*msg_type*/, const rclcpp::QoS & qos)
{
  tf2_subscriber_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    topic, qos,
    [this](
      const std::shared_ptr<const tf2_msgs::msg::TFMessage> & tfmsg) {
      this->tf2_callback(tfmsg);
    });
}

void SubscriptionManagerTF::tf2_callback(
  const std::shared_ptr<const tf2_msgs::msg::TFMessage> & tfmsg)
{
  for (size_t i = 0; i < tfmsg->transforms.size(); i++) {
    const geometry_msgs::msg::TransformStamped t = tfmsg->transforms[i];
    auto id = std::make_pair(t.header.frame_id, t.child_frame_id);
    auto it = tf_id_.find(id);
    if (it == tf_id_.end()) {
      tf_id_[id] = tfs_.transforms.size();
      tfs_.transforms.push_back(t);
      RCLCPP_INFO(
        node_->get_logger(), "TF list contains %lu transforms",
        tfs_.transforms.size());
    } else {
      tfs_.transforms[it->second] = t;
    }
  }
  std::shared_ptr<rclcpp::SerializedMessage> serialized_msg(new rclcpp::SerializedMessage);
  tf2_serialization_.serialize_message(&tfs_, serialized_msg.get());
  callback(serialized_msg);
}
