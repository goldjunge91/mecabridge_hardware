// Copyright (c) 2024 MecaBridge
// Apache-2.0

#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <string>

namespace mecabridge
{
namespace serial
{
class LoopbackSerialBackend
{
public:
  LoopbackSerialBackend() = default;
  ~LoopbackSerialBackend() = default;

  void open(const std::string & port, unsigned int baud) { (void)port; (void)baud; }
  void close() {}

  void write(const std::vector<uint8_t> & data)
  {
    std::unique_lock<std::mutex> lk(mutex_);
    for (auto b : data) buffer_.push_back(b);
    cv_.notify_all();
  }

  std::vector<uint8_t> read_some(size_t max_bytes, unsigned int timeout_ms)
  {
    std::unique_lock<std::mutex> lk(mutex_);
    if (buffer_.empty()) {
      if (timeout_ms == 0) return {};
      cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms));
    }
    size_t n = std::min(max_bytes, buffer_.size());
    std::vector<uint8_t> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      out.push_back(buffer_.front()); buffer_.pop_front();
    }
    return out;
  }

  size_t available()
  {
    std::unique_lock<std::mutex> lk(mutex_);
    return buffer_.size();
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<uint8_t> buffer_;
};

} // namespace serial
} // namespace mecabridge
