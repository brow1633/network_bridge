#pragma once

#include <list>
#include <mutex>
#include <condition_variable>

#include <network_bridge/thrift_stream.hpp>

namespace network_bridge
{

class QueueStream : public Stream
{
protected:
  bool finish;
  std::mutex mtx;
  std::condition_variable q_condition;
  std::list<uint8_t> Q;

  uint8_t getOneByte(std::unique_lock<std::mutex> & lock)
  {
    if (finish) {
      return 0;
    } else {
      if (Q.empty()) {
        q_condition.wait(lock);
        if (finish) {
          return 0;
        }
      }
      uint8_t v = Q.front();
      Q.pop_front();
      return v;
    }
  }

  uint8_t getOneByte()
  {
    if (finish) {
      return 0;
    } else {
      std::unique_lock<std::mutex> lock(mtx);
      if (Q.empty()) {
        q_condition.wait(lock);
        if (finish) {
          return 0;
        }
      }
      uint8_t v = Q.front();
      Q.pop_front();
      return v;
    }
  }

public:
  QueueStream()
  : finish(false)
  {
  }

  virtual ~QueueStream()
  {
    shutdown();
  }

  bool is_shutdown() const
  {
    return finish;
  }

  virtual int available()
  {
    std::unique_lock<std::mutex> lock(mtx);
    return Q.size();
  }

  void reset()
  {
    std::unique_lock<std::mutex> lock(mtx);
    Q.clear();
    finish = false;
    R.clear();
    recording = false;
  }

  void shutdown()
  {
    std::unique_lock<std::mutex> lock(mtx);
    finish = true;
    q_condition.notify_all();
    Q.clear();
    recording = false;
    R.clear();
  }

  template<typename iterator>
  bool pushBytes(iterator begin, iterator end)
  {
    std::unique_lock<std::mutex> lock(mtx);
    while (begin != end) {
      Q.push_back(*begin);
      begin++;
    }
    q_condition.notify_all();
    return true;
  }


  bool pushUint8(uint8_t v)
  {
    std::unique_lock<std::mutex> lock(mtx);
    Q.push_back(v);
    q_condition.notify_all();
    return true;
  }

  bool pushUint16(uint16_t v)
  {
    std::unique_lock<std::mutex> lock(mtx);
    Q.push_back((v >> 8) & 0xFF);
    Q.push_back(v & 0xFF);
    q_condition.notify_all();
    return true;
  }

  bool pushUint32(uint32_t v)
  {
    std::unique_lock<std::mutex> lock(mtx);
    Q.push_back((v >> 24) & 0xFF);
    Q.push_back((v >> 16) & 0xFF);
    Q.push_back((v >> 8) & 0xFF);
    Q.push_back(v & 0xFF);
    q_condition.notify_all();
    return true;
  }

  bool pushUint64(uint64_t v)
  {
    std::unique_lock<std::mutex> lock(mtx);
    Q.push_back((v >> 56) & 0xFF);
    Q.push_back((v >> 48) & 0xFF);
    Q.push_back((v >> 40) & 0xFF);
    Q.push_back((v >> 32) & 0xFF);
    Q.push_back((v >> 24) & 0xFF);
    Q.push_back((v >> 16) & 0xFF);
    Q.push_back((v >> 8) & 0xFF);
    Q.push_back(v & 0xFF);
    q_condition.notify_all();
    return true;
  }

  bool pushString(const std::string & v)
  {
    std::unique_lock<std::mutex> lock(mtx);
    for (size_t i = 0; i < v.size(); i++) {
      Q.push_back(uint8_t(v[i]));
    }
    q_condition.notify_all();
    return true;
  }


  virtual bool skipToNextMessage()
  {
    uint8_t c0 = 0, c1 = 0, c2 = 0;
    std::unique_lock<std::mutex> lock(mtx);
    c0 = getOneByte(lock);
    c1 = getOneByte(lock);
    c2 = getOneByte(lock);
    while ((c0 != 0x80) || (c1 != 0x01) || (c2 != 0x00)) {
      if (finish) {break;}
      if (recording) {
        R.push_back(c0);
      }
      c0 = c1;
      c1 = c2;
      c2 = getOneByte(lock);
    }
    Q.push_front(c2);
    Q.push_front(c1);
    Q.push_front(c0);
    return true;
  }


  virtual bool readBytes(std::vector<uint8_t> & bytes, size_t len)
  {
    bytes.clear();
    for (size_t i = 0; i < len; i++) {
      uint8_t b = getOneByte();
      bytes.push_back(b);
      if (recording) {
        R.push_back(b);
      }
    }
    return true;
  }

  virtual size_t readSome(std::vector<uint8_t> & bytes, size_t maxlen)
  {
    std::unique_lock<std::mutex> lock(mtx);
    size_t len = 0;
    while (!Q.empty() && ((maxlen == 0) || (len < maxlen))) {
      uint8_t v = Q.front();
      Q.pop_front();
      bytes.push_back(v);
      len += 1;
    }
    return len;
  }
};

}
