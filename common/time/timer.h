#pragma once

#include <chrono>
#include <string>

#include "common/macro/macros.h"

namespace common {

class Timer {
 public:
  Timer() {
    Tic();
  }

  void Tic();

  // unit: 's' = second, 'm' = millisecond, 'u' = microsecond
  double Toc(char unit = 'm');

 private:
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
  DISALLOW_COPY_AND_ASSIGN(Timer);
};

class TimerWrapper {
 public:
  explicit TimerWrapper(const std::string &msg, double duration_ms = -1.0)
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

#define BLOCK_TIMER_START ::common::Timer __timer__

#define BLOCK_TIMER_STOP __timer__.Toc()

#define BLOCK_TIMER_STOP_FMT FMT_TIMESTAMP(__timer__.Toc()) << " ms"

#define BLOCK_TIME_ELAPSED(msg) ::common::TimerWrapper __timer_wrapper__(msg)

#define BLOCK_TIME_ELAPSED_EXPECT(msg, max_time) \
  ::common::TimerWrapper __timer_wrapper__(msg, max_time)

}  // namespace common