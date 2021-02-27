#include "timer.h"

#include "common/log/log.h"

namespace common {

void Timer::Tic() {
  start_ = std::chrono::system_clock::now();
}

double Timer::Toc(char unit) {
  ACHECK(unit == 's' || unit == 'm' || unit == 'u')
      << "Only 's'(second), 'm'(millisecond) and 'u'(microsecond) are "
         "supported";
  double factor = 1.0;
  if (unit == 'm') factor = 1.0e3;
  if (unit == 'u') factor = 1.0e6;
  end_ = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_ - start_;
  return elapsed_seconds.count() * factor;
}

TimerWrapper::~TimerWrapper() {
  double duration = timer_.Toc();
  if (duration_ms_ < 0) {
    AINFO << msg_ << ": time elapsed: " << FMT_TIMESTAMP(duration) << " ms";
  }
  if (duration_ms_ > 0 && duration > duration_ms_) {
    AWARN << msg_ << ": time elapsed: " << FMT_TIMESTAMP(duration) << " ms";
  }
}

}  // namespace common