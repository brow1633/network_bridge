// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#include <rclcpp/serialization.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <span>
#include <zstd.h>

#include "ros2_tether/ros2_tether.hpp"
#include "network_interfaces/network_interface_base.hpp"

Ros2Tether::Ros2Tether(const std::string & node_name) 
    : Node(node_name), loader_("ros2_tether", "ros2_tether::NetworkInterface") {};

void Ros2Tether::initialize()
{
    load_parameters();
    load_network_interface();
    network_interface_->open();
}

void Ros2Tether::load_parameters()
{
    this->declare_parameter("network_interface", std::string("ros2_tether::UdpInterface"));
    this->get_parameter("network_interface", network_interface_name_);

    bool publish_stale_data;
    this->declare_parameter("publish_stale_data", false);
    this->get_parameter("publish_stale_data", publish_stale_data);
    // Defaults
    this->declare_parameter("default_rate", 5.0);
    this->declare_parameter("default_zstd_level", 3);

    float default_rate;
    int default_zstd_level;
    this->get_parameter("default_rate", default_rate);
    this->get_parameter("default_zstd_level", default_zstd_level);

    this->declare_parameter("namespace", "");
    this->get_parameter("namespace", namespace_);
    if (!namespace_.empty()) {
        if (namespace_.front() != '/') {
            namespace_.insert(0, "/");
        }
        if (namespace_.back() == '/') {
            namespace_.pop_back();
        }
        RCLCPP_INFO(this->get_logger(), "Topics will be published under the namespace %s", namespace_.c_str());
    }

    // Load topics information
    this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
    std::vector<std::string> topics;
    this->get_parameter("topics", topics);

    for (const auto& topic : topics) {
        std::string rate_param_name = topic + ".rate";
        std::string zstd_level_param_name = topic + ".zstd_level";

        this->declare_parameter<double>(rate_param_name, default_rate);
        this->declare_parameter<int>(zstd_level_param_name, default_zstd_level);

        float rate;
        int zstd_level;

        this->get_parameter(rate_param_name, rate);
        this->get_parameter(zstd_level_param_name, zstd_level);

        auto manager = std::make_shared<SubscriptionManager>(shared_from_this(), topic, zstd_level, publish_stale_data);
        sub_mgrs_.push_back(manager);

        int ms = static_cast<int>(1000.0 / rate);
        auto timer = this->create_wall_timer(std::chrono::milliseconds(ms), [this, manager]() {
            send_data(manager);
        });

        timers_.push_back(timer);

        RCLCPP_INFO(this->get_logger(), "Topic: %s, Rate: %f Hz", topic.c_str(), rate);
    }
}

void Ros2Tether::load_network_interface()
{
    try {
        network_interface_ = loader_.createSharedInstance(network_interface_name_);
        network_interface_->initialize(shared_from_this(), std::bind(&Ros2Tether::receive_data, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Loaded network interface: %s", network_interface_name_.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load network interface: %s", ex.what());
        rclcpp::shutdown();
        exit(1);
    }
}
void Ros2Tether::receive_data(std::span<const uint8_t> data)
{
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

    // Decompress data
    std::vector<uint8_t> decompressed_data;
    decompress(data, decompressed_data);

    std::string topic;
    std::string type;
    double current_time;
    parse_header(decompressed_data, topic, type, current_time);

    if(topic.empty() || type.empty())
    {
        return;
    }

    int header_length = sizeof(current_time) + topic.size() + 1 + type.size() + 1;

    std::span<const uint8_t> payload(decompressed_data.begin() + header_length, decompressed_data.end());

    float delay = rclcpp::Clock().now().seconds() - current_time;
    RCLCPP_DEBUG(this->get_logger(), "Received %lu bytes on topic %s with type %s", data.size(), topic.c_str(), type.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Decompressed data size: %lu", decompressed_data.size());
    RCLCPP_DEBUG(this->get_logger(), "Delay: %f ms", delay*1000);

    if(publishers_.find(topic) == publishers_.end())
    {
        // Create a QoS configuration with reliability and durability settings
        rclcpp::QoS qos(10);

        // Set QoS to Reliable
        qos.reliable();

        // Set QoS to Transient Local Durability
        qos.transient_local();
        publishers_[topic] = this->create_generic_publisher(namespace_ + topic, type, qos);
    }

    rclcpp::SerializedMessage msg(payload.size());
    std::copy(payload.begin(), payload.end(), msg.get_rcl_serialized_message().buffer);
    msg.get_rcl_serialized_message().buffer_length = payload.size();
    publishers_[topic]->publish(msg);

    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    RCLCPP_DEBUG(this->get_logger(), "Receive time: %f ms", std::chrono::duration<double, std::milli>(end - now).count());
}

void Ros2Tether::send_data(std::shared_ptr<SubscriptionManager> manager)
{
    const std::vector<uint8_t>& data = manager->get_data();

    if(data.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "SubscriptionManager %s has no data", manager->topic_.c_str());
        return;
    }

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    const std::string &topic = manager->topic_;
    const std::string &type = manager->msg_type_;

    std::vector<uint8_t> header;
    create_header(topic, type, header);

    // Form message
    std::vector<uint8_t> message;
    message.reserve(header.size() + data.size());
    message.insert(message.end(), header.begin(), header.end());
    message.insert(message.end(), data.begin(), data.end());

    // Compress data
    std::vector<uint8_t> compressed_data;
    compress(message, compressed_data, manager->zstd_compression_level_);

    // Send data
    network_interface_->write(compressed_data);
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    RCLCPP_DEBUG(this->get_logger(), "Send time: %f ms", std::chrono::duration<double, std::milli>(end - now).count());
}

void Ros2Tether::create_header(const std::string &topic, const std::string &msg_type, std::vector<uint8_t> &header)
{
    double current_time = rclcpp::Clock().now().seconds();

    header.clear();
    int header_length = sizeof(current_time) + topic.size() + 1 + msg_type.size() + 1;
    header.reserve(header_length);

    auto current_time_ptr = reinterpret_cast<uint8_t*>(&current_time);
    header.insert(header.end(), current_time_ptr, current_time_ptr + sizeof(current_time));
    
    header.insert(header.end(), topic.begin(), topic.end());
    header.push_back('\0');

    header.insert(header.end(), msg_type.begin(), msg_type.end());
    header.push_back('\0');
}

void Ros2Tether::parse_header(const std::vector<uint8_t> &header, std::string &topic, std::string &msg_type, double &time)
{
    // Add 4 for minimum usable header size (1 char for topic, 1 for msg_type, and 2 null terminators)
    if (header.size() < sizeof(time) + 4)
    {
        RCLCPP_ERROR(this->get_logger(), "Malformed header!");
        return;
    }

    time = *reinterpret_cast<const double*>(header.data());
    topic = reinterpret_cast<const char*>(header.data() + sizeof(time));
    msg_type = reinterpret_cast<const char*>(header.data() + sizeof(time) + topic.size() + 1);
}

void Ros2Tether::compress(std::vector<uint8_t> const& data, std::vector<uint8_t> &compressed_data, int zstd_compression_level)
{

    size_t compressedCapacity = ZSTD_compressBound(data.size());

    // Resize the output buffer to the capacity needed
    compressed_data.resize(compressedCapacity);

    // Compress the data
    size_t compressedSize = ZSTD_compress(compressed_data.data(), compressedCapacity, data.data(), data.size(), zstd_compression_level);

    // Check for errors
    if (ZSTD_isError(compressedSize)) {
        throw std::runtime_error(ZSTD_getErrorName(compressedSize));
    }

    // Resize compressed_data to actual compressed size
    compressed_data.resize(compressedSize);
}

void Ros2Tether::decompress(std::span<const uint8_t> compressed_data, std::vector<uint8_t> &data)
{
    // Find the size of the original uncompressed data
    unsigned long long decompressedSize = ZSTD_getFrameContentSize(compressed_data.data(), compressed_data.size());

    // Check if the size is known and valid
    if (decompressedSize == ZSTD_CONTENTSIZE_ERROR) {
        throw std::runtime_error("Not compressed by Zstd");
    } else if (decompressedSize == ZSTD_CONTENTSIZE_UNKNOWN) {
        throw std::runtime_error("Original size unknown");
    }

    // Resize the output buffer to the size of the uncompressed data
    data.resize(decompressedSize);

    // Decompress the data
    size_t decompressedResult = ZSTD_decompress(data.data(), decompressedSize, compressed_data.data(), compressed_data.size());

    // Check for errors during decompression
    if (ZSTD_isError(decompressedResult)) {
        throw std::runtime_error(ZSTD_getErrorName(decompressedResult));
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    // Randomized name to avoid conflicts
    std::srand(std::time(nullptr));
    std::string node_name = "ros2_tether" + std::to_string(std::rand());
    auto node = std::make_shared<Ros2Tether>(node_name);

    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
