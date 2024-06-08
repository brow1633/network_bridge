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

#include <span>
#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace network_bridge
{

/**
 * @class NetworkInterface
 * @brief Abstract base class for network interfaces.
 *
 * This class defines the interface for network communication. It provides methods for opening and closing the connection,
 * writing data, and setting a receive callback function.
 */
class NetworkInterface
{
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
  void initialize(
    const rclcpp::Node::SharedPtr & node,
    std::function<void(std::span<const uint8_t>)> recv_cb)
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
  virtual void write(const std::vector<uint8_t> & data) = 0;

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

}  // namespace network_bridge
