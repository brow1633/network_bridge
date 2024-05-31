// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#include <rclcpp/qos.hpp>
#include "ros2_tether/subscription_manager.hpp"

SubscriptionManager::SubscriptionManager(const rclcpp::Node::SharedPtr &  node, const std::string &topic, 
    const std::string &subscribe_namespace, int zstd_compression_level, bool publish_stale_data) :
    node_(node),
    msg_type_(),
    topic_(topic),
    subscribe_namespace_(subscribe_namespace),
    zstd_compression_level_(zstd_compression_level),
    received_msg_(false),
    is_stale_(true),
    publish_stale_data_(publish_stale_data),
    data_()
{
    setup_subscription();
}

void SubscriptionManager::setup_subscription()
{
    const auto all_topics_and_types = node_->get_topic_names_and_types();
    
    std::string topic = subscribe_namespace_ + topic_;

    if(all_topics_and_types.find(topic) == all_topics_and_types.end())
    {
        RCLCPP_DEBUG(node_->get_logger(), "Topic %s not found", topic.c_str());
        return;
    }

    // Get QoS profile from topic
    auto topic_info = node_->get_publishers_info_by_topic(topic);

    if(topic_info.size() == 0)
    {
        RCLCPP_DEBUG(node_->get_logger(), "No publishers found for topic %s", topic.c_str());
        return;
    }

    auto qos_candidate = topic_info[0].qos_profile().get_rmw_qos_profile();

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qos_candidate));

    qos.durability(qos_candidate.durability);
    qos.reliability(qos_candidate.reliability);

    if (qos_candidate.deadline.sec < 10000) 
    {
        qos.deadline(rclcpp::Duration(qos_candidate.deadline.sec, qos_candidate.deadline.nsec));
    }

    qos.liveliness(qos_candidate.liveliness);

    if (qos_candidate.liveliness_lease_duration.sec < 10000) 
    {
        qos.liveliness_lease_duration(rclcpp::Duration(qos_candidate.liveliness_lease_duration.sec, qos_candidate.liveliness_lease_duration.nsec));
    }

    msg_type_ = all_topics_and_types.at(topic)[0];

    subscriber = node_->create_generic_subscription(topic, msg_type_, qos, 
        [this](const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg) {
            this->callback(serialized_msg);
        }
    );
}

void SubscriptionManager::callback(const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg)
{
    RCLCPP_DEBUG(node_->get_logger(), "Received message on topic %s", topic_.c_str());
    received_msg_ = true;
    is_stale_ = false;
    data_.resize(serialized_msg->size());
    std::copy(serialized_msg->get_rcl_serialized_message().buffer, 
              serialized_msg->get_rcl_serialized_message().buffer + serialized_msg->size(), data_.begin());
}

const std::vector<uint8_t>& SubscriptionManager::get_data()
{
    if(!subscriber)
    {
        setup_subscription();
        RCLCPP_DEBUG(node_->get_logger(), "Send Timer: Subscriber is not set");
        return data_;
    }
    if(!received_msg_)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Send Timer: No message ever received");
        return data_;
    }
    if(is_stale_ && !publish_stale_data_)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Send Timer: Stored data is stale");
        data_.clear();
        return data_;
    }

    is_stale_ = true;
    return data_;
}