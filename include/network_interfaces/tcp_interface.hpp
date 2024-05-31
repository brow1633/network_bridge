// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#ifndef TCP_INTERFACE_HPP
#define TCP_INTERFACE_HPP

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include "network_interfaces/network_interface_base.hpp"

namespace ros2_tether {

using namespace boost::asio;

/**
 * @class TcpInterface
 * @brief Represents a TCP network interface.
 * 
 * The TcpInterface class is a concrete implementation of the NetworkInterface abstract class.
 * It provides functionality for opening, closing, receiving and writing data to a TCP interface.
 * It also handles receiving data asynchronously and provides error handling capabilities.
 */
class TcpInterface : public NetworkInterface {
public:
    TcpInterface() : 
        NetworkInterface(), socket_(io_context_), acceptor_(io_context_) {}

    virtual ~TcpInterface() 
    {
        close();
    }

protected:
    /**
     * @brief Initializes interface by loading parameters.
     * 
     * Called from NetworkInterface::initialize()
     */
    void initialize_() override;

public:
    void open() override;
    void close() override;
    void write(const std::vector<uint8_t>& data) override;

protected:
    void load_parameters();
    void setup_server();
    void setup_client();

    void start_receive();
    void receive(const boost::system::error_code& ec, std::size_t bytes_recvd);

    /**
     * @brief Handles errors from the UDP interface.
     * 
     * @param ec The error code from the operation.
     * @param error_message The error message to log.
     * @param fatal Whether the error is fatal.
     */
    void error_handler(const boost::system::error_code& ec, const std::string & error_message, bool fatal = false);
private:
    std::string role_;
    std::string remote_address_;
    int port_;

    io_context io_context_;
    ip::tcp::socket socket_;
    ip::tcp::acceptor acceptor_;
    std::thread io_thread_;

};
    
} // namespace ros2_tether

#endif // TCP_INTERFACE_HPP
