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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <network_bridge/subscription_manager.hpp>

/**
 * @class SubscriptionManager
 * @brief Manages and stores data of subscriptions to a specific topic.
 *
 * The SubscriptionManager class is responsible for managing and storing data of subscriptions to a specific topic.
 * It provides methods to retrieve the stored data and set up subscriptions for the topic.
 */
class SubscriptionManagerTF : public SubscriptionManager
{
public:
  /**
   * @brief Constructs a SubscriptionManager object.
   *
   * This constructor initializes a SubscriptionManager object with the given parameters.
   *
   * @param node A pointer to the rclcpp::Node object.
   * @param topic The topic to subscribe to.
   * @param zstd_compression_level The compression level for Zstandard compression (default: 3).
   * @param namespace The namespace for the subscription.
   * @param publish_stale_data Flag indicating whether to publish stale data (default: false).
   */
  SubscriptionManagerTF(
    const rclcpp::Node::SharedPtr & node, const std::string & topic,
    const std::string & subscribe_namespace, int zstd_compression_level = 3,
    bool publish_stale_data = false, bool static_tf = false);

  virtual ~SubscriptionManagerTF();

protected:
  void create_subscription(
    const std::string & topic,
    const std::string & msg_type, const rclcpp::QoS & qos) override;

  void tf2_callback(
    const std::shared_ptr<const tf2_msgs::msg::TFMessage> & tfmsg);

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf2_subscriber_;

  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf2_serialization_;
  std::map<std::pair<std::string, std::string>, size_t> tf_id_;
  tf2_msgs::msg::TFMessage tfs_;

  bool static_tf_;
};
