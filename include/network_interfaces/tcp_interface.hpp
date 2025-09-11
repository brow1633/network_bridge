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

#pragma once

#include <vector>
#include <string>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include "network_interfaces/network_interface_base.hpp"
#include "network_bridge/thrift_stream_queue.hpp"

namespace network_bridge
{

using boost::asio::ip::address;
using boost::asio::ip::tcp;
using boost::asio::io_context;

/**
 * @class TcpInterface
 * @brief Represents a TCP network interface.
 *
 * The TcpInterface class is a concrete implementation of the NetworkInterface abstract class.
 * It provides functionality for opening, closing, receiving and writing data to a TCP interface.
 * It also handles receiving data asynchronously and provides error handling capabilities.
 */
class TcpInterface : public NetworkInterface
{
public:
  TcpInterface()
  : NetworkInterface()
  {
    ready_ = false;
    failed_ = false;
  }

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
  bool has_failed() const override;
  bool is_ready() const override;
  void open() override;
  void close() override;
  void write(const std::vector<uint8_t> & data) override;

protected:
  void load_parameters();
  void setup_server();
  void setup_client();

  void start_receive();
  void receive(const boost::system::error_code & error, size_t rlen);
  void receive_thread();

  /**
   * @brief Handles errors from the UDP interface.
   *
   * @param ec The error code from the operation.
   * @param error_message The error message to log.
   * @param fatal Whether the error is fatal.
   */
  void error_handler(
    const boost::system::error_code & ec, const std::string & error_message,
    bool fatal = false);

private:
  std::string role_;
  std::string remote_address_;
  int port_;
  bool ready_;
  bool failed_;

  io_context io_context_;
  std::shared_ptr<tcp::socket> socket_;
  std::shared_ptr<tcp::acceptor> acceptor_;
  std::thread io_thread_;
  std::thread packet_thread_;

  network_bridge::QueueStream stream_;

};

}  // namespace network_bridge
