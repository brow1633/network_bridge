// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#ifndef UDP_SENDER_RECEIVER_HPP
#define UDP_SENDER_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

#include "ros2_tether/subscription_manager.hpp"
#include "network_interfaces/network_interface_base.hpp"

/**
 * @class Ros2Tether
 * @brief A class that represents a ROS 2 tether for communication over network.
 *
 * The `Ros2Tether` class is derived from the `rclcpp::Node` class and provides functionality for sending and receiving telemetry data over network.
 * It handles the setup of a network interface, parsing and creating headers, compressing and decompressing data, and error handling.
 * The class also manages the ROS 2 subscriptions, timers, and publishers associated with the communication.
 */
class Ros2Tether : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a Ros2Tether object.
     *
     * This constructor initializes a Ros2Tether object with the specified node name.
     *
     * @param node_name The name of the ROS 2 node.
     */
    Ros2Tether(const std::string & node_name);

    /**
     * @brief Loads parameters, loads network interface, and opens the network interface.
     * 
     * It should be called before any other functions are called on the object.
     */
    virtual void initialize();

protected:
    /**
     * @brief Loads default parameters and creates subsciption managers for each topic.
     */
    virtual void load_parameters();

    /**
     * @brief Loads the network interface as a dynamic plugin and initializes it.
     */
    virtual void load_network_interface();

    /**
     * @brief Callback function for handling received data.
     *
     * @param data The data to be received, represented as a span.
     */
    virtual void receive_data(std::span<const uint8_t> data);

    /**
     * @brief Sends data to the network interface.
     *
     * @param manager A shared pointer to the SubscriptionManager object.
     */
    virtual void send_data(std::shared_ptr<SubscriptionManager> manager);

    /**
     * @brief Creates a header for the message.
     *
     * @param topic The topic of the message.
     * @param msg_type The type of the message.
     * @param header The header to be created.
     */
    virtual void create_header(const std::string &topic, const std::string &msg_type, std::vector<uint8_t> &header);

    /**
     * @brief Parses the header of the message.
     *
     * @param header The header to be parsed.
     * @param topic The topic of the message.
     * @param msg_type The type of the message.
     * @param time The time the message was sent.
     */
    virtual void parse_header(const std::vector<uint8_t> &header, std::string &topic, std::string &msg_type, double &time);

    /**
     * Compresses the given data using Zstandard compression algorithm.
     *
     * @param data The input data to be compressed.
     * @param compressed_data [out] The vector to store the compressed data.
     * @param zstd_compression_level The compression level to be used (default: 3).
     */
    virtual void compress(std::vector<uint8_t> const& data, std::vector<uint8_t> &compressed_data, int zstd_compression_level = 3);

    /**
     * @brief Decompresses the given compressed data.
     *
     * This function takes a span of compressed data and decompresses it, storing the result in the provided data vector.
     *
     * @param compressed_data The compressed data to be decompressed.
     * @param data [out] The vector to store the decompressed data.
     */
    virtual void decompress(std::span<const uint8_t> compressed_data, std::vector<uint8_t> &data);

    /**
     * @brief A class template that provides a plugin loader for network interfaces.
     *
     * @tparam InterfaceT The interface type that the loaded plugins must implement.
     */
    pluginlib::ClassLoader<ros2_tether::NetworkInterface> loader_;

    /**
     * @brief The name of the network interface plugin.
     */
    std::string network_interface_name_;

    /**
     * @brief A shared pointer to an instance of the `ros2_tether::NetworkInterface` class.
     */
    std::shared_ptr<ros2_tether::NetworkInterface> network_interface_;

    /**
     * @brief A vector of the SubscriptionManager's for each topic.
     *
     * These are stored to keep them from going out of scope, but are not used directly.
     */
    std::vector<std::shared_ptr<SubscriptionManager>> sub_mgrs_;

    /**
     * @brief A vector of timers for sending each received topic over network.
     * 
     * These are stroed to keep them from going out of scope.
     * 
     * @see rclcpp::TimerBase
     */
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;

    /**
     * @brief A map that stores the publisher object against the topic name.
     */
    std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;

    /**
     * @brief the namespace for the publishers.
     */
    std::string namespace_;
};

#endif // UDP_SENDER_RECEIVER_HPP
