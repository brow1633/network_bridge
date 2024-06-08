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

#include <boost/asio/steady_timer.hpp>

#include "network_interfaces/tcp_interface.hpp"

namespace network_bridge
{

void TcpInterface::initialize_()
{
  load_parameters();
}

void TcpInterface::load_parameters()
{
  std::string prefix = "TcpInterface.";
  node_->declare_parameter(prefix + "role", std::string(""));
  node_->declare_parameter(prefix + "remote_address", std::string(""));
  node_->declare_parameter(prefix + "port", 0);

  node_->get_parameter(prefix + "role", role_);
  node_->get_parameter(prefix + "remote_address", remote_address_);
  node_->get_parameter(prefix + "port", port_);

  RCLCPP_INFO(node_->get_logger(), "role_: %s", role_.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "Remote Address: %s", remote_address_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Remote Port: %d", port_);
}

void TcpInterface::open()
{
  if (role_ == "server") {
    setup_server();
  } else if (role_ == "client") {
    setup_client();
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid role specified: %s", role_.c_str());
  }

  io_thread_ = std::thread(
    [this]() {
      io_context_.run();
    });
}

void TcpInterface::close()
{
  io_context_.stop();
  io_thread_.join();
}

void TcpInterface::setup_server()
{
  boost::system::error_code ec;
  bool fatal = true;

  tcp::endpoint endpoint(tcp::v4(), port_);

  acceptor_.open(endpoint.protocol(), ec);
  error_handler(ec, "Failed to open acceptor", fatal);

  acceptor_.set_option(tcp::acceptor::reuse_address(true), ec);
  error_handler(ec, "Failed to set acceptor option", fatal);

  acceptor_.bind(endpoint, ec);
  error_handler(ec, "Failed to bind acceptor", fatal);

  acceptor_.listen(tcp::socket::max_listen_connections, ec);
  error_handler(ec, "Failed to listen on acceptor", fatal);

  acceptor_.async_accept(
    socket_,
    [this](const boost::system::error_code & ec) {
      error_handler(ec, "Failed to accept connection", true);
      RCLCPP_INFO(
        node_->get_logger(),
        "Accepted connection from %s:%u",
        socket_.remote_endpoint().address().to_string().c_str(),
        socket_.remote_endpoint().port());
      start_receive();
    });
}

void TcpInterface::setup_client()
{
  boost::system::error_code ec;
  bool fatal = true;

  tcp::endpoint endpoint(
    address::from_string(remote_address_), port_);
  error_handler(
    ec, "Failed to connect to server: check address and port.", fatal);


  while (true) {
    socket_.connect(endpoint, ec);
    if (!ec) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Connected to server at %s:%u",
        remote_address_.c_str(), port_);
      start_receive();
      break;
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "Failed to connect to server, retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

void TcpInterface::error_handler(
  const boost::system::error_code & ec,
  const std::string & error_message,
  bool fatal)
{
  if (ec) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: %s",
      error_message.c_str(), ec.message().c_str());

    if (fatal) {
      RCLCPP_FATAL(node_->get_logger(), "fatal error, shutting down");
      rclcpp::shutdown();
      exit(1);
    }
  }
}

void TcpInterface::start_receive()
{
  // async_receive will stop after filling buffer
  // So we can receive a header followed by a payload
  // note: this is an additional header for message size specific to TCP
  auto header_buffer = boost::asio::buffer(receive_buffer_, sizeof(size_t));
  socket_.async_receive(
    header_buffer,
    [this](const boost::system::error_code & ec, std::size_t bytes_recvd) {
      if (!ec && bytes_recvd == sizeof(size_t)) {
        size_t payload_size = *reinterpret_cast<size_t *>(receive_buffer_.data());
        receive(payload_size);
      } else {
        error_handler(ec, "Failed to receive header");
      }
    });
}

void TcpInterface::receive(size_t payload_size)
{
  auto body_buffer = boost::asio::buffer(receive_buffer_, payload_size);
  socket_.async_receive(
    body_buffer,
    [this, payload_size](const boost::system::error_code & ec, std::size_t bytes_recvd) {
      if (!ec && bytes_recvd == payload_size) {
        recv_cb_(std::span<uint8_t>(receive_buffer_.data(), bytes_recvd));
        start_receive();
      } else {
        error_handler(ec, "Failed to receive body");
      }
    });
}

void TcpInterface::write(const std::vector<uint8_t> & data)
{
  boost::system::error_code ec;

  // Sync write to block thread from sending multiple messages at once
  size_t size = data.size();
  socket_.write_some(boost::asio::buffer(&size, sizeof(size)), ec);
  socket_.write_some(boost::asio::buffer(data, size), ec);
  error_handler(ec, "Failed to write data");
}

}  // namespace network_bridge

PLUGINLIB_EXPORT_CLASS(network_bridge::TcpInterface, network_bridge::NetworkInterface)
