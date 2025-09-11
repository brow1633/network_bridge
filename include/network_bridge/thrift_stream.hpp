#pragma once

#include <stdint.h>
#include <vector>
#include <list>
#include <string>

namespace network_bridge
{

class Stream
{
public:
  Stream() {}
  virtual ~Stream() {}

  virtual void shutdown() = 0;

  virtual bool skipToNextMessage() = 0;

  virtual bool readBytes(std::vector<uint8_t> & bytes, size_t len) = 0;

  virtual size_t readSome(std::vector<uint8_t> & bytes, size_t maxlen) = 0;

  virtual int available()
  {
    return -1;             // not implemented
  }

protected:
  bool recording;
  std::list<uint8_t> R;           // byte recording

public:
  void startRecording()
  {
    R.clear();
    recording = true;
    // std::cout << "Start recording " << std::endl;
  }

  void stopRecording()
  {
    recording = false;
    // std::cout << "Stop recording: " << R.size() << std::endl;
  }

  const std::list<uint8_t> getRecording() const
  {
    return R;
  }

public:
  bool readUint8(uint8_t & b)
  {
    std::vector<uint8_t> bytes;
    if (!readBytes(bytes, 1)) {
      return false;
    }
    b = bytes[0];
    return true;
  }

  bool readUint16(uint16_t & b)
  {
    std::vector<uint8_t> bytes;
    if (!readBytes(bytes, 2)) {
      return false;
    }
    b = bytes[0];
    b = (b << 8) | bytes[1];
    return true;
  }

  bool readUint32(uint32_t & b)
  {
    std::vector<uint8_t> bytes;
    if (!readBytes(bytes, 4)) {
      return false;
    }
    b = bytes[0];
    b = (b << 8) | bytes[1];
    b = (b << 8) | bytes[2];
    b = (b << 8) | bytes[3];
    return true;
  }

  bool readUint64(uint64_t & b)
  {
    std::vector<uint8_t> bytes;
    if (!readBytes(bytes, 8)) {
      return false;
    }
    b = bytes[0];
    for (int i = 1; i < 8; i++) {
      b = (b << 8) | bytes[i];
    }
    return true;
  }

  bool readString(std::string & s, size_t len)
  {
    std::vector<uint8_t> bytes;
    if (!readBytes(bytes, len)) {
      return false;
    }
    s.clear();
    for (size_t i = 0; i < len; i++) {
      s.push_back(char(bytes[i]));
    }
    return true;
  }
};


}
