#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <iostream>

// here we define G3LOG instead of use LOG directly, since LOG macro in g3log
// conflicts LOG macro in glog.
#define G3LOG(level) \
  if (g3::logLevel(level)) INTERNAL_LOG_MESSAGE(level).stream()

#define VARIABLE_WITH_LINE_NUM(var_name, line) var_name##line
#define COUNT_VAR_WITH_LINE_NUM VARIABLE_WITH_LINE_NUM(_count_var_, __LINE__)

#define G3LOG_EVERY(level, n)                                      \
  static size_t COUNT_VAR_WITH_LINE_NUM = 0;                       \
  if ((COUNT_VAR_WITH_LINE_NUM)++ % static_cast<size_t>(n) == 0 && \
      g3::logLevel(level))                                         \
    INTERNAL_LOG_MESSAGE(level).stream();

#define G3LOG_IF(level, boolean_expression)        \
  if ((boolean_expression) && g3::logLevel(level)) \
  INTERNAL_LOG_MESSAGE(level).stream()

#define G3CHECK(boolean_expression) \
  if (!(boolean_expression))        \
  INTERNAL_CONTRACT_MESSAGE(#boolean_expression).stream()

const LEVELS ERROR{WARNING.value + 100, "ERROR"};
const LEVELS USER(ERROR.value + 100, "USER");

// LOG
#define ADEBUG G3LOG(DEBUG)
#define AINFO G3LOG(INFO)
#define AWARN G3LOG(WARNING)
#define AERROR G3LOG(ERROR)
#define AUSER G3LOG(USER)
#define AFATAL G3LOG(FATAL)

// LOG_IF
#define ADEBUG_IF(cond) G3LOG_IF(DEBUG, cond)
#define AINFO_IF(cond) G3LOG_IF(INFO, cond)
#define AWARN_IF(cond) G3LOG_IF(WARNING, cond)
#define AERROR_IF(cond) G3LOG_IF(ERROR, cond)
#define AUSER_IF(cond) G3LOG_IF(USER, cond)
#define AFATAL_IF(cond) G3LOG_IF(FATAL, cond)
#define ACHECK(cond) G3CHECK(cond)

// LOG_EVERY
#define ADEBUG_EVERY(n) G3LOG_EVERY(DEBUG, n)
#define AINFO_EVERY(n) G3LOG_EVERY(INFO, n)
#define AWARN_EVERY(n) G3LOG_EVERY(WARNING, n)
#define AERROR_EVERY(n) G3LOG_EVERY(ERROR, n)
#define AUSER_EVERY(n) G3LOG_EVERY(USER, n)

namespace common {
void InitG3Logging(bool log_to_file = false, const std::string &prefix = "",
                   const std::string &path = "./");
}  // namespace common

namespace g3 {
class CustomSink {
 public:
  CustomSink() = default;

  explicit CustomSink(const std::string &log_file_name)
      : log_file_name_(log_file_name), log_to_file_(true) {
    ofs_.reset(new std::ofstream(log_file_name));
  }

  ~CustomSink();

  void StdLogMessage(g3::LogMessageMover log_entry) {
    std::clog << ColorFormatedMessage(log_entry.get()) << std::endl;
  }

  void FileLogMessage(g3::LogMessageMover log_entry) {
    if (log_to_file_) {
      (*ofs_) << FormatedMessage(log_entry.get()) << std::endl;
    }
  }

 private:
  std::string log_file_name_;
  bool log_to_file_{false};
  std::unique_ptr<std::ofstream> ofs_{nullptr};

  std::string FormatedMessage(const g3::LogMessage &msg) const;

  std::string ColorFormatedMessage(const g3::LogMessage &msg) const;

  std::string GetColorCode(const LEVELS &level) const;
};

}  // namespace g3