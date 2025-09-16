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
#include <regex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <network_bridge/subscription_manager.hpp>

/**
 * @class SubscriptionManagerTF
 * @brief Manages and stores data of subscriptions to a specific TF topic.
 *
 * The SubscriptionManager class is responsible for managing and storing data of subscriptions to a specific TF topic.
 * It provides methods to manage the transforms, avoiding any missed transform
 */
class SubscriptionManagerTF : public SubscriptionManager
{
public:
  /**
   * @brief Constructs a SubscriptionManagerTF object.
   *
   * This constructor initializes a SubscriptionManager object with the given parameters.
   *
   * @param node A pointer to the rclcpp::Node object.
   * @param topic The topic to subscribe to.
   * @param zstd_compression_level The compression level for Zstandard compression (default: 3).
   * @param namespace The namespace for the subscription.
   * @param publish_stale_data Flag indicating whether to publish stale data (default: false).
   * @param static_tf Flag indicating whether the subscriber is a static transform .
   */
  SubscriptionManagerTF(
    const rclcpp::Node::SharedPtr & node, const std::string & topic,
    const std::string & subscribe_namespace, int zstd_compression_level = 3,
    bool publish_stale_data = false, bool static_tf = false);

  virtual ~SubscriptionManagerTF();

  /**
   * @brief Check if the subscription has been successful, or try to set it up
   *
   */
  void check_subscription() override;

  /**
   * @brief Check if the subscriber data is stale, but returns always false for static_tf
   *
   */
  bool is_stale() const override;

  /**
   * @brief Store the vector of tf name include pattern, and convert them to std::regex
   *
   * Note: a transform is matched if either the frame_id or the child_frame_id match the regex.
   *
   */
  void set_include_pattern(const std::vector<std::string> & pattern);

  /**
   * @brief Store the vector of tf name exclude pattern, and convert them to std::regex
   *
   * Note: a transform is matched if either the frame_id or the child_frame_id match the regex.
   *
   */
  void set_exclude_pattern(const std::vector<std::string> & pattern);

protected:
  /**
   * @brief Create the subscriber
   *
   * This function creates the actual subscriber after setup-subscription has
   * handled the qos and other params. Can be overloaded by specialized
   * subscribers
   */
  void create_subscription(
    const std::string & topic,
    const std::string & msg_type, const rclcpp::QoS & qos) override;

  /**
   * @brief Callback function for handling tf2 messages.
   *
   * This function is called when a tf2 message is received by the subscription manager.
   * It stores the recovered transforms in the tfs_ messages
   *
   * @param tfmsg A shared pointer to the tf2 message.
   */
  void tf2_callback(
    const std::shared_ptr<const tf2_msgs::msg::TFMessage> & tfmsg);

  /**
   * @brief The ROS2 TF2 subscriber object.
   */
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf2_subscriber_;

  /**
   * @brief The ROS2 TF2 serialization object.
   */
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf2_serialization_;
  /**
   * @brief Map linking (frame_id,child_frame_id) to the position in tf_s
   */
  std::map<std::pair<std::string, std::string>, size_t> tf_id_;
  /**
   * @brief TF message grouping all the transforms received so far.
   */
  tf2_msgs::msg::TFMessage tfs_;

  /**
   * @brief Flag indicating if this a static tf topic
   */
  bool static_tf_;

  /**
   * @brief List of accepted tf name pattern (frame_id or child), ignored if empty
   */
  std::vector<std::regex> include_pattern;
  /**
   * @brief List of excluded tf name pattern (frame_id or child), ignored if empty
   */
  std::vector<std::regex> exclude_pattern;
};
