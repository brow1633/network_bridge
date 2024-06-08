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

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "network_bridge/subscription_manager.hpp"
#include "network_interfaces/network_interface_base.hpp"

/**
 * @class NetworkBridge
 * @brief A class that provides bridging of ROS2 topics over a network interface.
 *
 * The `NetworkBridge` class is derived from the `rclcpp::Node` class and provides functionality for sending and receiving telemetry data over network.
 * It handles the setup of a network interface, parsing and creating headers, compressing and decompressing data, and error handling.
 * The class also manages the ROS 2 subscriptions, timers, and publishers associated with the communication.
 */
class NetworkBridge : public rclcpp::Node
{
public:
  /**
   * @brief Constructs a NetworkBridge object.
   *
   * This constructor initializes a NetworkBridge object with the specified node name.
   *
   * @param node_name The name of the ROS 2 node.
   */
  explicit NetworkBridge(const std::string & node_name);

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
  virtual std::vector<uint8_t> create_header(
    const std::string & topic, const std::string & msg_type);

  /**
   * @brief Parses the header of the message.
   *
   * @param header The header to be parsed.
   * @param topic The topic of the message.
   * @param msg_type The type of the message.
   * @param time The time the message was sent.
   */
  virtual void parse_header(
    const std::vector<uint8_t> & header, std::string & topic,
    std::string & msg_type, double & time);

  /**
   * Compresses the given data using Zstandard compression algorithm.
   *
   * @param data The input data to be compressed.
   * @param compressed_data [out] The vector to store the compressed data.
   * @param zstd_compression_level The compression level to be used (default: 3).
   */
  virtual void compress(
    std::vector<uint8_t> const & data, std::vector<uint8_t> & compressed_data,
    int zstd_compression_level = 3);

  /**
   * @brief Decompresses the given compressed data.
   *
   * This function takes a span of compressed data and decompresses it,
   * storing the result in the provided data vector
   *
   * @param compressed_data The compressed data to be decompressed.
   * @param data [out] The vector to store the decompressed data.
   */
  virtual void decompress(
    std::span<const uint8_t> compressed_data, std::vector<uint8_t> & data);

  /**
   * @brief A class template that provides a plugin loader for network interfaces.
   *
   * @tparam InterfaceT The interface type that the loaded plugins must implement.
   */
  pluginlib::ClassLoader<network_bridge::NetworkInterface> loader_;

  /**
   * @brief The name of the network interface plugin.
   */
  std::string network_interface_name_;

  /**
   * @brief A shared pointer to an instance of the `network_bridge::NetworkInterface` class.
   */
  std::shared_ptr<network_bridge::NetworkInterface> network_interface_;

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
  std::string publish_namespace_;
};
