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

#include "network_interfaces/zmq_interface.hpp"

namespace network_bridge {

void ZmqInterface::initialize_() { load_parameters(); }

void ZmqInterface::load_parameters() {
  std::string prefix = "ZmqInterface.";
  node_->declare_parameter(prefix + "role", std::string(""));
  node_->declare_parameter(prefix + "remote_address", std::string(""));
  node_->declare_parameter(prefix + "port", 0);

  node_->get_parameter(prefix + "role", role_);
  node_->get_parameter(prefix + "remote_address", remote_address_);
  node_->get_parameter(prefix + "port", port_);

  RCLCPP_INFO(node_->get_logger(), "role_: %s", role_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Remote Address: %s",
              remote_address_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Remote Port: %d", port_);
}

void ZmqInterface::open() {
  shutting_down_ = false;
  failed_ = false;
  ready_ = false;
  if (role_ == "server") {
    setup_server();
    ready_ = true;
  } else if (role_ == "client") {
    setup_client();
    ready_ = true;
    packet_thread_ =
        std::thread(std::bind(&ZmqInterface::receive_thread, this));
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid role specified: %s",
                 role_.c_str());
    failed_ = true;
    return;
  }
}

bool ZmqInterface::is_ready() const { return ready_ && !failed_; }

bool ZmqInterface::has_failed() const { return failed_; }

void ZmqInterface::close() {
  if (shutting_down_.exchange(true)) {
    return;
  }
  ready_ = false;

  if (socket_) {
    try {
      socket_->close();
    } catch (const zmqpp::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "ZMQ exception: %s", e.what());
    }
  }

  if (packet_thread_.joinable()) {
    packet_thread_.join();
  }

  try {
    context_.terminate();
  } catch (const zmqpp::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "ZMQ exception: %s", e.what());
  }
}

void ZmqInterface::receive_thread() {
  zmqpp::poller poller;
  poller.add(*socket_, zmqpp::poller::poll_in);
  while (!shutting_down_ && rclcpp::ok()) {
    if (poller.poll(100)) {
      zmqpp::message msg;
      socket_->receive(msg);
      const void *data = msg.raw_data(0);
      size_t size = msg.size(0);
      recv_cb_(
          std::span<const uint8_t>(static_cast<const uint8_t *>(data), size));
    }
  }
}

void ZmqInterface::setup_server() {
  socket_ = std::make_shared<zmqpp::socket>(context_, zmqpp::socket_type::push);
  socket_->bind("tcp://*:" + std::to_string(port_));
}

void ZmqInterface::setup_client() {
  socket_ = std::make_shared<zmqpp::socket>(context_, zmqpp::socket_type::pull);
  socket_->connect("tcp://" + remote_address_ + ":" + std::to_string(port_));
}

void ZmqInterface::write(const std::vector<uint8_t> &data) {
  zmqpp::message msg;
  msg.add_raw(data.data(), data.size());
  socket_->send(msg);
}

} // namespace network_bridge

PLUGINLIB_EXPORT_CLASS(network_bridge::ZmqInterface,
                       network_bridge::NetworkInterface)
