#include <chrono>
#include <memory>
#include <stdexcept>

#include <boost/asio.hpp>
#include <gtest/gtest.h>

#include "network_interfaces/zmq_interface.hpp"

namespace
{

class ZmqInterfaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  int unused_port()
  {
    boost::asio::io_context io;
    boost::asio::ip::tcp::acceptor acceptor(io, {boost::asio::ip::tcp::v4(), 0});
    return acceptor.local_endpoint().port();
  }

  rclcpp::Node::SharedPtr node(const std::string & role, const std::string & pattern, int port)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {
        rclcpp::Parameter("ZmqInterface.role", role),
        rclcpp::Parameter("ZmqInterface.pattern", pattern),
        rclcpp::Parameter("ZmqInterface.port", port),
        rclcpp::Parameter("ZmqInterface.remote_address", "127.0.0.1")});
    return std::make_shared<rclcpp::Node>("zmq_interface_test", options);
  }
};

TEST_F(ZmqInterfaceTest, ServerCanCloseAndReopen)
{
  for (const auto & pattern : {"pub_sub", "push_pull"}) {
    SCOPED_TRACE(pattern);
    network_bridge::ZmqInterface interface;
    interface.initialize(node("server", pattern, unused_port()), [](auto) {});
    interface.open();
    ASSERT_TRUE(interface.is_ready());
    interface.close();
    EXPECT_FALSE(interface.is_ready());
    interface.open();
    EXPECT_TRUE(interface.is_ready());
    EXPECT_FALSE(interface.has_failed());
    interface.close();
    interface.close();
  }
}

TEST_F(ZmqInterfaceTest, AbsentPushPeerDoesNotBlockWriteOrShutdown)
{
  network_bridge::ZmqInterface interface;
  interface.initialize(node("server", "push_pull", unused_port()), [](auto) {});
  interface.open();
  const auto start = std::chrono::steady_clock::now();
  interface.write({1, 2, 3});
  interface.close();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(2));
}

TEST_F(ZmqInterfaceTest, ClientCanCloseAndReopenWithoutServer)
{
  network_bridge::ZmqInterface interface;
  interface.initialize(node("client", "pub_sub", unused_port()), [](auto) {});
  interface.open();
  interface.close();
  interface.open();
  EXPECT_TRUE(interface.is_ready());
  // Receive-only clients must not send on their worker's socket.
  interface.write({1});
  interface.close();
}

TEST_F(ZmqInterfaceTest, InvalidConfigurationIsRejected)
{
  network_bridge::ZmqInterface interface;
  EXPECT_THROW(
    interface.initialize(node("server", "typo", 5555), [](auto) {}), std::invalid_argument);
}

TEST_F(ZmqInterfaceTest, BindFailureNeverReportsReady)
{
  const int port = unused_port();
  network_bridge::ZmqInterface first;
  network_bridge::ZmqInterface second;
  first.initialize(node("server", "pub_sub", port), [](auto) {});
  second.initialize(node("server", "pub_sub", port), [](auto) {});
  first.open();
  ASSERT_TRUE(first.is_ready());
  second.open();
  EXPECT_FALSE(second.is_ready());
  EXPECT_TRUE(second.has_failed());
}

}  // namespace
