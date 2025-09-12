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
  failed_ = false;
  ready_ = false;
  io_context_.restart();
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

  packet_thread_ = std::thread(std::bind(&TcpInterface::receive_thread, this));
}

bool TcpInterface::is_ready() const
{
  return ready_ && !failed_;
}

bool TcpInterface::has_failed() const
{
  return failed_;
}


void TcpInterface::close()
{
  if (acceptor_) {
    acceptor_->close();
  }
  acceptor_.reset();
  if (socket_) {
    socket_->close();
  }
  socket_.reset();
  stream_.shutdown();
  try {
    packet_thread_.join();
  } catch (std::system_error &) {
  }
  io_context_.stop();
  try {
    io_thread_.join();
  } catch (std::system_error &) {
  }
}

void TcpInterface::receive_thread()
{
  uint8_t state = 0;
  uint8_t start1, start2;
  uint32_t payload_size;
  std::vector<uint8_t> payload;
  while (!stream_.is_shutdown() && rclcpp::ok()) {
    switch (state) {
      case 0:
        if (!stream_.readUint8(start1)) {
          RCLCPP_ERROR(node_->get_logger(), "Receive thread could not receive first start packet");
          break;
        }
        if (start1 == 0xAB) {
          state = 1;
        }
        break;
      case 1:
        if (!stream_.readUint8(start2)) {
          RCLCPP_ERROR(node_->get_logger(), "Receive thread could not receive second start packet");
          state = 0;
          break;
        }
        if (start2 == 0xCD) {
          state = 2;
        } else if (start2 == 0xAB) {
          state = 1;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "invalid second start packet");
          state = 0;
        }
        break;
      case 2:
        payload_size = 0;
        if (!stream_.readUint32(payload_size)) {
          RCLCPP_ERROR(node_->get_logger(), "Receive thread could not receive payload size");
          state = 0;
          break;
        }
        payload_size = htonl(payload_size);
        state = 3;
        break;
      case 3:
        payload.clear();
        if (!stream_.readBytes(payload, payload_size)) {
          RCLCPP_ERROR(node_->get_logger(), "Receive thread could not receive payload");
          state = 0;
          break;
        }
        recv_cb_(std::span<uint8_t>(payload.data(), payload_size));
        state = 0;
        break;
      default:
        state = 0;
    }
  }
}

void TcpInterface::setup_server()
{
  boost::system::error_code ec;
  bool fatal = true;

  tcp::endpoint endpoint(tcp::v4(), port_);

  socket_.reset(new tcp::socket(io_context_));
  acceptor_.reset(new tcp::acceptor(io_context_));

  acceptor_->open(endpoint.protocol(), ec);
  error_handler(ec, "Failed to open acceptor", fatal);

  acceptor_->set_option(tcp::acceptor::reuse_address(true), ec);
  error_handler(ec, "Failed to set acceptor option", fatal);

  acceptor_->bind(endpoint, ec);
  error_handler(ec, "Failed to bind acceptor", fatal);

  acceptor_->listen(tcp::socket::max_listen_connections, ec);
  error_handler(ec, "Failed to listen on acceptor", fatal);

  RCLCPP_INFO(node_->get_logger(), "Accepting connections");
  acceptor_->async_accept(
    *socket_,
    [this](const boost::system::error_code & ec) {
      error_handler(ec, "Failed to accept connection", true);
      RCLCPP_INFO(
        node_->get_logger(),
        "Accepted connection from %s:%u",
        socket_->remote_endpoint().address().to_string().c_str(),
        socket_->remote_endpoint().port());
      ready_ = true;
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


  socket_.reset(new tcp::socket(io_context_));
  while (rclcpp::ok()) {
    socket_->connect(endpoint, ec);
    if (!ec) {
      RCLCPP_INFO(
        node_->get_logger(), "Connected to server at %s:%u",
        remote_address_.c_str(), port_);
      ready_ = true;
      start_receive();
      break;
    } else {
      ready_ = false;
      RCLCPP_INFO(node_->get_logger(), "Failed to connect to server, retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
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
  socket_->async_read_some(
    boost::asio::buffer(receive_buffer_),
    boost::bind(
      &TcpInterface::receive, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void TcpInterface::receive(const boost::system::error_code & error, size_t rlen)
{
  if (!error) {
    stream_.pushBytes(receive_buffer_.begin(), receive_buffer_.begin() + rlen);
    start_receive();
  } else {
    failed_ = true;
    error_handler(error, "Failed to receive buffer");
    // std::cerr << "out->in: connection error" << std::endl;
  }
}

void TcpInterface::write(const std::vector<uint8_t> & data)
{
  try {
    // Sync write to block thread from sending multiple messages at once
    uint8_t start[2] = {0xAB, 0xCD};
    boost::asio::write(*socket_, boost::asio::buffer(start, 2));
    uint32_t size = data.size();
    boost::asio::write(*socket_, boost::asio::buffer(&size, sizeof(size)));
    boost::asio::write(*socket_, boost::asio::buffer(data, size));
  } catch (std::exception &) {
    failed_ = true;
    RCLCPP_WARN(node_->get_logger(), "Exception while writing data");
  }
}

}  // namespace network_bridge

PLUGINLIB_EXPORT_CLASS(network_bridge::TcpInterface, network_bridge::NetworkInterface)
