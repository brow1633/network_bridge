// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#ifndef UDP_SENDER_RECEIVER_HPP
#define UDP_SENDER_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include "basestation_com_v2/subscription_manager.hpp"

using namespace boost::asio;
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
    Ros2Tether(const std::string & node_name);
    void init();

private:
    void load_parameters();
    void setup_udp();

    void receive_telemetry(const boost::system::error_code& ec, std::size_t bytes_recvd);
    void send_telemetry(std::shared_ptr<SubscriptionManager> ros2udp_interface);
    void start_receive();

    void create_header(const std::string &topic, const std::string &msg_type, std::vector<uint8_t> &header);
    void parse_header(const std::vector<uint8_t> &header, std::string &topic, std::string &msg_type, double &time);
    void compress(std::vector<uint8_t> const& data, std::vector<uint8_t> &compressed_data, int zstd_compression_level = 3);
    void decompress(std::span<const uint8_t> compressed_data, std::vector<uint8_t> &data);

    void error_handler(const boost::system::error_code& ec, const std::string & error_message, bool fatal = false)
    {
        if(ec)
        {
            RCLCPP_ERROR(this->get_logger(), "%s: %s", error_message.c_str(), ec.message().c_str());

            if(fatal)
            {
                RCLCPP_FATAL(this->get_logger(), "Fatal error, shutting down");
                rclcpp::shutdown();
                exit(1);
            }
        }
    }
    // UDP Stuff
    io_service io_service_;
    std::thread io_thread_;

    ip::udp::socket send_telemetry_socket_;
    ip::udp::endpoint send_telemetry_endpoint_;
    ip::udp::socket receive_telemetry_socket_;
    ip::udp::endpoint receive_telemetry_endpoint_;
    ip::udp::endpoint receive_endpoint_;
    std::string local_address_;
    int receive_port_;
    std::string remote_address_;
    int send_port_;

    std::vector<uint8_t> receive_buffer_;

    std::vector<std::shared_ptr<SubscriptionManager>> ros2udp_interfaces_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    std::map<std::string, rclcpp::GenericPublisher::SharedPtr> udp_publishers_;

    // Zstd Stuff
    std::string zstd_dictionary_file_;
    std::vector<uint8_t> zstd_dictionary_;
    int zstd_dict_size_;
    bool train_zstd_dictionary_;
    std::vector<std::vector<uint8_t>> zstd_training_data_;
};

#endif // UDP_SENDER_RECEIVER_HPP
