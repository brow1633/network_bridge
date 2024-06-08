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

#include "network_interfaces/udp_interface.hpp"

namespace network_bridge
{

void UdpInterface::initialize_()
{
  load_parameters();
}

void UdpInterface::open()
{
  setup_udp();
  io_thread_ = std::thread(
    [this]() {
      io_context_.run();
    });
}

void UdpInterface::close()
{
  io_context_.stop();
  io_thread_.join();
}

void UdpInterface::load_parameters()
{
  std::string prefix = "UdpInterface.";
  node_->declare_parameter(prefix + "local_address", std::string(""));
  node_->declare_parameter(prefix + "receive_port", 0);
  node_->declare_parameter(prefix + "remote_address", std::string(""));
  node_->declare_parameter(prefix + "send_port", 0);

  node_->get_parameter(prefix + "local_address", local_address_);
  node_->get_parameter(prefix + "receive_port", receive_port_);
  node_->get_parameter(prefix + "remote_address", remote_address_);
  node_->get_parameter(prefix + "send_port", send_port_);

  RCLCPP_INFO(
    node_->get_logger(),
    "Local Address: %s", local_address_.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "Receive Port: %d", receive_port_);
  RCLCPP_INFO(
    node_->get_logger(),
    "Remote Address: %s", remote_address_.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "Send Port: %d", send_port_);
}

void UdpInterface::setup_udp()
{
  boost::system::error_code ec;
  bool fatal = true;

  // Setup sending socket
  send_socket_.open(udp::v4(), ec);
  error_handler(ec, "Failed to open sending socket", fatal);

  RCLCPP_INFO(
    node_->get_logger(),
    "Setting up remote endpoint with address: %s and port: %u",
    remote_address_.c_str(), send_port_);

  send_endpoint_ = udp::endpoint(
    address::from_string(remote_address_, ec), send_port_);
  error_handler(ec, "Failed to parse remote address", fatal);

  RCLCPP_INFO(
    node_->get_logger(),
    "Established connection to send data to: %s:%u",
    remote_address_.c_str(), send_port_);

  // Setup receiving socket
  receive_socket_.open(udp::v4(), ec);
  error_handler(ec, "Failed to open receiving socket", fatal);

  RCLCPP_INFO(
    node_->get_logger(),
    "Setting up local endpoint with address: %s and port: %u",
    local_address_.c_str(), receive_port_);

  receive_endpoint_ = udp::endpoint(
    address::from_string(local_address_, ec), receive_port_);
  error_handler(ec, "Failed to parse local address", fatal);

  receive_socket_.bind(receive_endpoint_, ec);
  error_handler(ec, "Failed to bind receiving socket", fatal);

  start_receive();
}

void UdpInterface::error_handler(
  const boost::system::error_code & ec,
  const std::string & error_message,
  bool fatal)
{
  if (ec) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: %s",
      error_message.c_str(), ec.message().c_str());

    if (fatal) {
      RCLCPP_FATAL(node_->get_logger(), "Fatal error, shutting down");
      rclcpp::shutdown();
      exit(1);
    }
  }
}

void UdpInterface::start_receive()
{
  receive_socket_.async_receive_from(
    boost::asio::buffer(receive_buffer_), receive_endpoint_,
    boost::bind(
      &UdpInterface::receive, this, boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void UdpInterface::receive(
  const boost::system::error_code & ec,
  std::size_t bytes_recvd)
{
  if (bytes_recvd <= 0) {
    start_receive();
    return;
  }

  if (!ec) {
    recv_cb_(std::span<const uint8_t>(receive_buffer_.data(), bytes_recvd));
    start_receive();
  } else {
    error_handler(ec, "Failed to receive data");
  }
}

void UdpInterface::write(const std::vector<uint8_t> & data)
{
  if (!send_socket_.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Socket is not open");
    return;
  }

  boost::system::error_code ec;
  send_socket_.send_to(boost::asio::buffer(data), send_endpoint_, 0, ec);
  error_handler(ec, "Failed to send data");
}

}  // namespace network_bridge

PLUGINLIB_EXPORT_CLASS(network_bridge::UdpInterface, network_bridge::NetworkInterface)
