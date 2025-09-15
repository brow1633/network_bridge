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
  }

  void shutdown()
  {
    std::unique_lock<std::mutex> lock(mtx);
    finish = true;
    q_condition.notify_all();
    Q.clear();
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


  virtual bool readBytes(std::vector<uint8_t> & bytes, size_t len)
  {
    std::unique_lock<std::mutex> lock(mtx);
    bytes.clear();
    while (bytes.size() < len) {
      if (Q.empty()) {
        q_condition.wait(lock);
        if (finish) {
          return false;
        }
      }
      bytes.push_back(Q.front());
      Q.pop_front();
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
