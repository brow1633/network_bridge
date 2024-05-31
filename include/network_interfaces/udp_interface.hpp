// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ethan M Brown
// This file is part of ROS2 Tether, distributed under the MIT License.
// For full terms, see the included LICENSE.txt file.

#ifndef UDP_INTERFACE_HPP
#define UDP_INTERFACE_HPP

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include "network_interfaces/network_interface_base.hpp"

namespace ros2_tether {

using namespace boost::asio;

/**
 * @class UdpInterface
 * @brief Represents a UDP network interface.
 * 
 * The UdpInterface class is a concrete implementation of the NetworkInterface abstract class.
 * It provides functionality for opening, closing, receiving and writing data to a UDP interface.
 * It also handles receiving data asynchronously and provides error handling capabilities.
 */
class UdpInterface : public NetworkInterface {
public:
    UdpInterface() : 
        NetworkInterface(), send_socket_(io_context_), receive_socket_(io_context_) {}
    virtual ~UdpInterface() 
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
    /**
     * @brief Opens the UDP interface.
     * 
     * This function is responsible for opening the UDP interface and preparing it for communication.
     * It should be called before any other operations are performed on the interface.
     */
    void open() override;

    /**
     * @brief Closes the UDP interface.
     *
     * After calling this function, the UDP interface will no longer be usable.
     */
    void close() override;

    /**
     * @brief Writes the given data to the UDP interface.
     *
     * @param data The data to be written.
     */
    void write(const std::vector<uint8_t>& data) override;
protected:
    /**
     * @brief Starts receiving data on the UDP interface.
     * 
     * Re-binds the receive socket and starts an asynchronous receive operation.
     */
    void start_receive();

    /**
     * @brief Callback function for receiving data.
     * 
     * It calls the receive callback function with the received data and starts another receive operation.
     * 
     * @param ec The error code from the receive operation.
     * @param bytes_recvd The number of bytes received.
     */
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
    /**
     * @brief Loads parameters from the ROS 2 parameter server.
     */
    void load_parameters();

    /**
     * @brief Sets up the UDP interface.
     */
    void setup_udp();

protected:
    io_context io_context_;
    std::thread io_thread_;

    ip::udp::socket send_socket_;
    ip::udp::endpoint send_endpoint_;
    ip::udp::socket receive_socket_;
    ip::udp::endpoint receive_endpoint_;
    std::string local_address_;
    int receive_port_;
    std::string remote_address_;
    int send_port_;

};

} // namespace ros2_tether

#endif // UDP_INTERFACE_HPP
