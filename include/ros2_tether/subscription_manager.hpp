// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#ifndef __SUBSCRIPTION_MANAGER_HPP__
#define __SUBSCRIPTION_MANAGER_HPP__

#include <rclcpp/rclcpp.hpp>

/**
 * @class SubscriptionManager
 * @brief Manages and stores data of subscriptions to a specific topic.
 *
 * The SubscriptionManager class is responsible for managing and storing data of subscriptions to a specific topic.
 * It provides methods to retrieve the stored data and set up subscriptions for the topic.
 */
class SubscriptionManager
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
     * @param publish_stale_data Flag indicating whether to publish stale data (default: false).
     */
    SubscriptionManager(const rclcpp::Node::SharedPtr & node, const std::string & topic, int zstd_compression_level = 3, bool publish_stale_data = false);

    /**
     * @brief Retrieves the data stored in the subscription manager.
     * 
     * This method returns a constant reference to the vector containing the stored data.
     * If no data has been received or if the data is stale and the flag publish_stale_data_ is false,
     * an empty vector is returned.
     *  
     * @return A constant reference to the vector containing the data.
     */
    const std::vector<uint8_t>& get_data();

protected:
    /**
     * @brief Sets up a subscription for a given topic.
     *
     * This function sets up a subscription for the specified topic.
     * It takes a string parameter representing the topic to subscribe to.
     * This function is called automatically in the constructor and get_data() method.
     * It fails if the topic does not exist or if there are no publishers on this topic.
     * 
     * @param topic The topic to subscribe to.
     */
    void setup_subscription(const std::string & topic);

    /**
     * @brief Callback function for handling serialized messages.
     *
     * This function is called when a serialized message is received by the subscription manager.
     * It stores the data from serialized_msg into the data_ vector.
     *
     * @param serialized_msg A shared pointer to the serialized message.
     */
    void callback(const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg);

    /**
     * @brief Pointer to the ROS 2 node.
     */
    const rclcpp::Node::SharedPtr node_;

public:
    /**
     * @brief The type of message stored in the subscription manager.
     */
    std::string msg_type_;

    /**
     * @brief The topic name for the subscription.
     */
    std::string topic_;

    /**
     * @brief The compression level used for Zstandard compression (1->22).
     */
    int zstd_compression_level_;

protected:
    /**
     * @brief Flag indicating whether a message has ever been received.
     */
    bool received_msg_;

    /**
     * @brief Flag indicating whether the data is stale (already accessed via get_data()).
     */
    bool is_stale_;

    /**
     * @brief Flag indicating whether to publish stale data.
     */
    bool publish_stale_data_;

    /**
     * @brief The ROS2 generalized subscriber object.
     */
    rclcpp::GenericSubscription::SharedPtr subscriber;

    /**
     * @brief The data buffer for the subscription manager.
     */
    std::vector<uint8_t> data_;
};

#endif // __SUBSCRIPTION_MANAGER_HPP__