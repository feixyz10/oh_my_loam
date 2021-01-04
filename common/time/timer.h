#pragma once

#include <chrono>
#include <string>

#include "common/macro/macros.h"

namespace common {

class Timer {
 public:
  Timer() { Tic(); }

  void Tic() { start_ = std::chrono::system_clock::now(); }

  // unit: 's' = second, 'm' = millisecond, 'u' = microsecond
  double Toc(char unit = 'm');

 private:
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
  DISALLOW_COPY_AND_ASSIGN(Timer);
};

class TimerWrapper {
 public:
  explicit TimerWrapper(const std::string& msg, double duration_ms = -1.0)
      : msg_(msg), duration_ms_(duration_ms) {
    timer_.Tic();
  };

  ~TimerWrapper();

 private:
  std::string msg_;
  double duration_ms_;
  Timer timer_;
  DISALLOW_COPY_AND_ASSIGN(TimerWrapper);
};

#define TIME_ELAPSED(msg) TimerWrapper(msg)
#define TIME_ELAPSED_EXPECT(msg, max_time_elapsed) \
  TimerWrapper(msg, max_time_elapsed)

}  // namespace common