#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>

namespace network_bridge
{

// Preserve the existing wire format, including its native-endian timestamp.
inline bool parse_packet_header(
  std::span<const uint8_t> data, std::string & topic,
  std::string & type, double & timestamp)
{
  topic.clear();
  type.clear();
  timestamp = 0.0;
  if (data.size() < sizeof(timestamp) + 4) {
    return false;
  }
  const auto topic_begin = data.begin() + sizeof(timestamp);
  const auto topic_end = std::find(topic_begin, data.end(), 0);
  if (topic_end == data.end() || topic_end == topic_begin) {
    return false;
  }
  const auto type_begin = topic_end + 1;
  const auto type_end = std::find(type_begin, data.end(), 0);
  if (type_end == data.end() || type_end == type_begin) {
    return false;
  }
  std::memcpy(&timestamp, data.data(), sizeof(timestamp));
  topic.assign(topic_begin, topic_end);
  type.assign(type_begin, type_end);
  return true;
}

}  // namespace network_bridge
