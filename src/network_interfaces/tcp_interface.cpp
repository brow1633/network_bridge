// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#include <boost/asio/steady_timer.hpp>

#include "network_interfaces/tcp_interface.hpp"

namespace ros2_tether {

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
    RCLCPP_INFO(node_->get_logger(), "Remote Address: %s", remote_address_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Remote Port: %d", port_);
}

void TcpInterface::open()
{
    if(role_ == "server")
    {
        setup_server();
    }
    else if(role_ == "client")
    {
        setup_client();
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Invalid role specified: %s", role_.c_str());
    }

    io_thread_ = std::thread([this]() {
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

    ip::tcp::endpoint endpoint(ip::tcp::v4(), port_);

    acceptor_.open(endpoint.protocol(), ec);
    error_handler(ec, "Failed to open acceptor", fatal);

    acceptor_.set_option(ip::tcp::acceptor::reuse_address(true), ec);
    error_handler(ec, "Failed to set acceptor option", fatal);

    acceptor_.bind(endpoint, ec);
    error_handler(ec, "Failed to bind acceptor", fatal);

    acceptor_.listen(ip::tcp::socket::max_listen_connections, ec);
    error_handler(ec, "Failed to listen on acceptor", fatal);

    acceptor_.async_accept(socket_, [this](const boost::system::error_code& ec)
    {
        error_handler(ec, "Failed to accept connection", true);
        RCLCPP_INFO(node_->get_logger(), "Accepted connection from %s:%u", socket_.remote_endpoint().address().to_string().c_str(), socket_.remote_endpoint().port());
        start_receive();
    });

}

void TcpInterface::setup_client()
{
    boost::system::error_code ec;
    bool fatal = true;

    ip::tcp::endpoint endpoint(ip::address::from_string(remote_address_), port_);
    error_handler(ec, "Failed to connect to server: Unable to create endpoint, check address and port.", fatal);


    socket_.async_connect(endpoint, [this](const boost::system::error_code& ec)
    {
        steady_timer timer(io_context_);
        if(!ec)
        {
            RCLCPP_INFO(node_->get_logger(), "Connected to server at %s:%u", remote_address_.c_str(), port_);
            start_receive();
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Failed to connect to server, retrying...");
            timer.expires_after(std::chrono::milliseconds(100));
            timer.async_wait(std::bind(&TcpInterface::setup_client, this));
        }
    });
}

void TcpInterface::error_handler(const boost::system::error_code& ec, const std::string & error_message, bool fatal)
{
    if(ec)
    {
        RCLCPP_ERROR(node_->get_logger(), "%s: %s", error_message.c_str(), ec.message().c_str());

        if(fatal)
        {
            RCLCPP_FATAL(node_->get_logger(), "fatal error, shutting down");
            rclcpp::shutdown();
            exit(1);
        }
    }
}

void TcpInterface::start_receive()
{
    socket_.async_receive(buffer(receive_buffer_), [this](const boost::system::error_code& ec, std::size_t bytes_recvd)
    {
        receive(ec, bytes_recvd);
    });
}

void TcpInterface::receive(const boost::system::error_code& ec, std::size_t bytes_recvd)
{
    if(bytes_recvd <= 0)
    {
        start_receive();
        return;
    }

    if(!ec)
    {
        recv_cb_(std::span<const uint8_t>(receive_buffer_.data(), bytes_recvd));
        start_receive();
    }
    else
    {
        error_handler(ec, "Failed to receive data");
    }
}

void TcpInterface::write(const std::vector<uint8_t>& data)
{
    boost::system::error_code ec;

    socket_.write_some(buffer(data), ec);
    error_handler(ec, "Failed to write data");
}

} // namespace ros2_tether

PLUGINLIB_EXPORT_CLASS(ros2_tether::TcpInterface, ros2_tether::NetworkInterface)
