#ifndef DEBUG_SIGNALS_HPP
#define DEBUG_SIGNALS_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

struct PIDAxisDebug
{
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double error = 0.0;
  double control_sig = 0.0;
};

struct DebugSample
{
  std::uint64_t sequence = 0;
  std::chrono::steady_clock::time_point sample_time = std::chrono::steady_clock::now();

  std::array<double, 7> joint_position{};
  std::array<double, 7> joint_velocity{};

  std::array<double, 6> ee_vel{};
  std::array<double, 6> ee_vel_error{};
  std::array<double, 6> control_signal{};
  std::array<PIDAxisDebug, 6> pid_axes{};

  int fsm_state = 0;
};

class DebugSignalBuffer
{
public:
  explicit DebugSignalBuffer(std::size_t capacity = 2000)
  : capacity_(capacity)
  {}

  void push(const DebugSample& sample)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (capacity_ == 0) {
      return;
    }
    while (queue_.size() >= capacity_) {
      queue_.pop_front();
    }
    queue_.push_back(sample);
  }

  std::vector<DebugSample> drain(std::size_t max_count)
  {
    std::vector<DebugSample> out;
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t count = std::min(max_count, queue_.size());
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      out.push_back(queue_.front());
      queue_.pop_front();
    }
    return out;
  }

private:
  std::size_t capacity_;
  std::deque<DebugSample> queue_;
  std::mutex mutex_;
};

#endif // DEBUG_SIGNALS_HPP