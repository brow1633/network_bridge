// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#ifndef NETWORK_INTERFACE_HPP
#define NETWORK_INTERFACE_HPP

#include <functional>
#include <memory>
#include <span>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ros2_tether {

/**
 * @class NetworkInterface
 * @brief Abstract base class for network interfaces.
 * 
 * This class defines the interface for network communication. It provides methods for opening and closing the connection,
 * writing data, and setting a receive callback function.
 */
class NetworkInterface {
public:
    /**
     * @brief Constructor for NetworkInterface.
     */
    NetworkInterface() 
        : receive_buffer_(65536) {}

    /**
     * @brief Virtual destructor for NetworkInterface.
     */
    virtual ~NetworkInterface() = default;

    /**
     * @brief Initializes the network interface.
     *
     * This function sets the node and receive callback for the network interface, and then calls the protected
     * `initialize_` function to perform any additional initialization steps for a specific implementation.
     *
     * @param node A shared pointer to the ROS 2 node.
     * @param recv_cb The callback function to be called when data is received.
     */
    void initialize(const rclcpp::Node::SharedPtr& node, std::function<void(std::span<const uint8_t>)> recv_cb)
    {
        node_ = node;
        recv_cb_ = recv_cb;

        initialize_();
    }

protected:
    /**
     * @brief Initializes the network interface.
     * 
     * This function is pure virtual and must be implemented by derived classes.
     * Should declare and load all parameters, such as IP addresses, ports, etc.
     */
    virtual void initialize_() = 0;

public:
    /**
     * @brief Opens the network connection.
     * 
     * This method should be implemented by derived classes to establish the network connection.
     * Should handle all network setup, including sockets, starting threads, etc
     */
    virtual void open() = 0;

    /**
     * @brief Closes the network connection.
     * 
     * This method should be implemented by derived classes to close the network connection.
     * Cleanly stop threads, close sockets, etc.
     */
    virtual void close() = 0;

    /**
     * @brief Writes data to the network.
     * 
     * This method should be implemented by derived classes to send data over the network.
     * @param data The data to be written.
     */
    virtual void write(const std::vector<uint8_t>& data) = 0;


protected:
    /**
     * @brief Shared pointer to the ROS 2 node.
    */
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief The data buffer for received data.
     */
    std::vector<uint8_t> receive_buffer_;

    /**
     * @brief The receive callback function.
     */
    std::function<void(std::span<const uint8_t>)> recv_cb_;
};

} // namespace ros2_tether

#endif // NETWORK_INTERFACE_HPP