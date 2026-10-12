#include <chrono>
#include <cstring>
#include <future>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "network_bridge/packet_header.hpp"
#include "network_bridge/thrift_stream_queue.hpp"

namespace
{

class PacketHeaderTest : public ::testing::Test
{
protected:
  const double sent_time = 123456.75;
  std::vector<uint8_t> packet;
  std::string topic;
  std::string type;
  double timestamp = 0.0;

  void SetUp() override
  {
    packet.resize(sizeof(sent_time));
    std::memcpy(packet.data(), &sent_time, sizeof(sent_time));
    const char fields[] = "/test\0std_msgs/msg/String";
    packet.insert(packet.end(), fields, fields + sizeof(fields));
  }

  bool parse(std::span<const uint8_t> data)
  {
    return network_bridge::parse_packet_header(data, topic, type, timestamp);
  }
};

TEST_F(PacketHeaderTest, DecodesTimestampAndFields)
{
  ASSERT_TRUE(parse(packet));
  EXPECT_EQ(topic, "/test");
  EXPECT_EQ(type, "std_msgs/msg/String");
  EXPECT_DOUBLE_EQ(timestamp, sent_time);
}

TEST_F(PacketHeaderTest, RejectsEveryTruncatedHeaderAndClearsOutputs)
{
  for (size_t len = 0; len < packet.size(); ++len) {
    SCOPED_TRACE(len);
    topic = "previous topic";
    type = "previous type";
    timestamp = sent_time;
    EXPECT_FALSE(parse(std::span<const uint8_t>(packet.data(), len)));
    EXPECT_TRUE(topic.empty());
    EXPECT_TRUE(type.empty());
    EXPECT_DOUBLE_EQ(timestamp, 0.0);
  }
}

TEST_F(PacketHeaderTest, RejectsMissingTerminators)
{
  std::vector<uint8_t> malformed(100, 'x');
  EXPECT_FALSE(parse(malformed));
}

TEST_F(PacketHeaderTest, RejectsEmptyFields)
{
  packet[sizeof(double)] = 0;
  EXPECT_FALSE(parse(packet));
  SetUp();
  packet[sizeof(double) + sizeof("/test")] = 0;
  EXPECT_FALSE(parse(packet));
}

TEST_F(PacketHeaderTest, IgnoresBinaryPayloadAfterHeader)
{
  packet.insert(packet.end(), {0, 255, 0, 127});
  ASSERT_TRUE(parse(packet));
  EXPECT_EQ(type, "std_msgs/msg/String");
}

TEST(QueueStreamTest, PreservesByteOrderAcrossChunks)
{
  network_bridge::QueueStream stream;
  ASSERT_TRUE(stream.pushUint16(0x1234));
  ASSERT_TRUE(stream.pushUint16(0x5678));
  std::vector<uint8_t> bytes;
  ASSERT_TRUE(stream.readBytes(bytes, 4));
  EXPECT_EQ(bytes, (std::vector<uint8_t>{0x12, 0x34, 0x56, 0x78}));
}

TEST(QueueStreamTest, ShutdownRejectsReadsAndWrites)
{
  network_bridge::QueueStream stream;
  stream.shutdown();
  std::vector<uint8_t> bytes;
  EXPECT_FALSE(stream.readBytes(bytes, 1));
  EXPECT_FALSE(stream.pushUint8(1));
}

TEST(QueueStreamTest, ResetClearsOldBytesAndAllowsReuse)
{
  network_bridge::QueueStream stream;
  stream.pushUint8(1);
  stream.shutdown();
  stream.reset();
  EXPECT_FALSE(stream.is_shutdown());
  EXPECT_EQ(stream.available(), 0);
  ASSERT_TRUE(stream.pushUint8(2));
  uint8_t byte = 0;
  ASSERT_TRUE(stream.readUint8(byte));
  EXPECT_EQ(byte, 2);
}

TEST(QueueStreamTest, ShutdownReleasesReaderRegardlessOfScheduling)
{
  using namespace std::chrono_literals;
  network_bridge::QueueStream stream;
  auto reader = std::async(
    std::launch::async, [&stream]() {
      std::vector<uint8_t> bytes;
      return stream.readBytes(bytes, 1);
    });
  stream.shutdown();
  // The ament test timeout also bounds failures that prevent the worker exiting.
  ASSERT_EQ(reader.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(reader.get());
}

}  // namespace
