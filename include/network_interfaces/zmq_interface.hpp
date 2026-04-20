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

#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <zmqpp/zmqpp.hpp>

#include "network_interfaces/network_interface_base.hpp"

namespace network_bridge
{

/**
 * @class ZmqInterface
 * @brief Represents a ZMQ network interface.
 *
 * The ZmqInterface class is a concrete implementation of the NetworkInterface
 * abstract class. It provides functionality for opening, closing, receiving and
 * writing data to a ZMQ interface. It also handles receiving data
 * asynchronously and provides error handling capabilities.
 */
class ZmqInterface : public NetworkInterface
{
public:
  ZmqInterface()
  : NetworkInterface()
  {
    ready_ = false;
    failed_ = false;
  }

  virtual ~ZmqInterface() {close();}

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
  void receive_thread();

private:
  zmqpp::context context_;
  std::shared_ptr<zmqpp::socket> socket_;

  std::string role_;
  std::string remote_address_;
  int port_;
  std::atomic<bool> ready_;
  std::atomic<bool> failed_;
  std::atomic<bool> shutting_down_;

  std::thread packet_thread_;
};

} // namespace network_bridge
