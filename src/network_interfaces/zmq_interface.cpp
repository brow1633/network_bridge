/*
==============================================================================
MIT License

Copyright (c) 2024 Ethan M Brown
Copyright (c) 2026 PAL Robotics

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

#include <span>
#include <stdexcept>

#include "network_interfaces/zmq_interface.hpp"

namespace network_bridge
{

void ZmqInterface::initialize_() {load_parameters();}

void ZmqInterface::load_parameters()
{
  std::string prefix = "ZmqInterface.";
  node_->declare_parameter(prefix + "role", std::string(""));
  node_->declare_parameter(prefix + "pattern", std::string("pub_sub"));
  node_->declare_parameter(prefix + "remote_address", std::string(""));
  node_->declare_parameter(prefix + "port", 0);

  node_->get_parameter(prefix + "role", role_);
  node_->get_parameter(prefix + "pattern", pattern_);
  node_->get_parameter(prefix + "remote_address", remote_address_);
  node_->get_parameter(prefix + "port", port_);

  if (role_ != "server" && role_ != "client") {
    throw std::invalid_argument("ZmqInterface.role must be server or client");
  }
  if (pattern_ != "pub_sub" && pattern_ != "push_pull") {
    throw std::invalid_argument("ZmqInterface.pattern must be pub_sub or push_pull");
  }
  if (port_ < 1 || port_ > 65535) {
    throw std::invalid_argument("ZmqInterface.port must be in [1, 65535]");
  }
  if (role_ == "client" && remote_address_.empty()) {
    throw std::invalid_argument("ZmqInterface.remote_address is required for clients");
  }

  RCLCPP_INFO(node_->get_logger(), "role_: %s", role_.c_str());
  RCLCPP_INFO(node_->get_logger(), "pattern_: %s", pattern_.c_str());
  RCLCPP_INFO(
    node_->get_logger(), "Remote Address: %s",
    remote_address_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Remote Port: %d", port_);
}

void ZmqInterface::open()
{
  close();
  context_ = std::make_unique<zmq::context_t>();
  shutting_down_ = false;
  failed_ = false;
  ready_ = false;
  if (role_ == "server") {
    setup_server();
    ready_ = !failed_;
  } else if (role_ == "client") {
    setup_client();
    ready_ = !failed_;
    if (failed_) {return;}
    packet_thread_ =
      std::thread(std::bind(&ZmqInterface::receive_thread, this));
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Invalid role specified: %s",
      role_.c_str());
    failed_ = true;
    return;
  }
}

bool ZmqInterface::is_ready() const {return ready_ && !failed_;}

bool ZmqInterface::has_failed() const {return failed_;}

void ZmqInterface::close()
{
  if (shutting_down_.exchange(true)) {
    return;
  }
  ready_ = false;

  // The client socket belongs to the receive thread until it exits.
  if (packet_thread_.joinable()) {
    packet_thread_.join();
  }
  socket_.reset();
  // Context destruction waits for asynchronous socket cleanup before reopening.
  context_.reset();

}

void ZmqInterface::receive_thread()
{
  try {
    zmq::pollitem_t item{socket_->handle(), 0, ZMQ_POLLIN, 0};
    while (!shutting_down_ && rclcpp::ok()) {
      zmq::poll(&item, 1, std::chrono::milliseconds(100));
      if (item.revents & ZMQ_POLLIN) {
        zmq::message_t msg;
        if (!socket_->recv(msg, zmq::recv_flags::dontwait)) {continue;}
        // A bridge packet is exactly one frame. Drain malformed multipart
        // messages so their trailing frames cannot become separate packets.
        if (msg.more()) {
          do {
            if (!socket_->recv(msg, zmq::recv_flags::dontwait)) {break;}
          } while (msg.more());
          continue;
        }
        const void * data = msg.data();
        size_t size = msg.size();
        recv_cb_(
          std::span<const uint8_t>(static_cast<const uint8_t *>(data), size));
      }
    }
  } catch (const std::exception & e) {
    failed_ = true;
    ready_ = false;
    RCLCPP_ERROR(node_->get_logger(), "ZMQ receive failed: %s", e.what());
  }
}

void ZmqInterface::setup_server()
{
  zmq::socket_type type =
    (pattern_ == "pub_sub") ? zmq::socket_type::pub : zmq::socket_type::push;
  socket_ = std::make_unique<zmq::socket_t>(*context_, type);
  socket_->set(zmq::sockopt::linger, 0);
  try {
    socket_->bind("tcp://*:" + std::to_string(port_));
  } catch (const zmq::error_t & e) {
    RCLCPP_ERROR(node_->get_logger(), "Bind failed: %s", e.what());
    failed_ = true;
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Server bound to port %d", port_);
}

void ZmqInterface::setup_client()
{
  zmq::socket_type type =
    (pattern_ == "pub_sub") ? zmq::socket_type::sub : zmq::socket_type::pull;
  socket_ = std::make_unique<zmq::socket_t>(*context_, type);
  socket_->set(zmq::sockopt::linger, 0);
  if (pattern_ == "pub_sub") {
    socket_->set(zmq::sockopt::subscribe, "");
  }
  try {
    socket_->connect("tcp://" + remote_address_ + ":" + std::to_string(port_));
  } catch (const zmq::error_t & e) {
    RCLCPP_ERROR(node_->get_logger(), "Connect failed: %s", e.what());
    failed_ = true;
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Client connected to port %d", port_);
}

void ZmqInterface::write(const std::vector<uint8_t> & data)
{
  if (!is_ready() || role_ != "server") {return;}
  try {
    zmq::message_t msg(data.data(), data.size());
    // Do not stall the ROS executor when a PUSH peer is absent or congested.
    if (!socket_->send(msg, zmq::send_flags::dontwait)) {
      RCLCPP_WARN(node_->get_logger(), "ZMQ send queue unavailable; dropping sample");
    }
  } catch (const std::exception & e) {
    failed_ = true;
    ready_ = false;
    RCLCPP_ERROR(node_->get_logger(), "ZMQ send failed: %s", e.what());
  }
}

}  // namespace network_bridge

PLUGINLIB_EXPORT_CLASS(
  network_bridge::ZmqInterface,
  network_bridge::NetworkInterface)
