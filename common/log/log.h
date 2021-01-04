#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <iostream>

// here we define G3LOG instead of use LOG directly, since LOG macro in g3log
// conflicts LOG macro in glog. Same for G3LOG_IF and G3CHECK
#define G3LOG(level)          \
  if (!g3::logLevel(level)) { \
  } else                      \
  INTERNAL_LOG_MESSAGE(level).stream()

#define G3LOG_IF(level, boolean_expression)                    \
  if (false == (boolean_expression) || !g3::logLevel(level)) { \
  } else                                                       \
  INTERNAL_LOG_MESSAGE(level).stream()

#define G3CHECK(boolean_expression)   \
  if (true == (boolean_expression)) { \
  } else                              \
  INTERNAL_CONTRACT_MESSAGE(#boolean_expression).stream()

const LEVELS ERROR{WARNING.value + 100, "ERROR"};
const LEVELS USER(ERROR.value + 100, "USER");

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

namespace common {
void InitG3Logging(bool log_to_file = false, const std::string& prefix = "",
                   const std::string& path = "./");
}  // namespace common

namespace g3 {
class CustomSink {
 public:
  CustomSink() = default;

  explicit CustomSink(const std::string& log_file_name)
      : log_file_name_(log_file_name), log_to_file_(true) {
    ofs_.reset(new std::ofstream(log_file_name));
  }

  ~CustomSink() {
    std::ostringstream oss;
    oss << "\ng3log " << (log_to_file_ ? "FileSink" : "StdSink")
        << " shutdown at: ";
    auto now = std::chrono::system_clock::now();
    oss << g3::localtime_formatted(now, "%Y%m%d %H:%M:%S.%f3");
    if (log_to_file_) {
      (*ofs_) << oss.str() << std::endl;
    } else {
      std::clog << oss.str() << std::endl;
    }
  };

  void StdLogMessage(g3::LogMessageMover logEntry) {
    std::clog << ColorFormatedMessage(logEntry.get()) << std::endl;
  }

  void FileLogMessage(g3::LogMessageMover logEntry) {
    if (log_to_file_) {
      (*ofs_) << FormatedMessage(logEntry.get()) << std::endl;
    }
  }

 private:
  std::string log_file_name_;
  bool log_to_file_{false};
  std::unique_ptr<std::ofstream> ofs_{nullptr};

  std::string FormatedMessage(const g3::LogMessage& msg) const {
    std::ostringstream oss;
    oss << "[" << msg.level()[0] << msg.timestamp("%Y%m%d %H:%M:%S.%f3") << " "
        << msg.file() << ":" << msg.line() << "] " << msg.message();
    return oss.str();
  }

  std::string ColorFormatedMessage(const g3::LogMessage& msg) const {
    std::ostringstream oss;
    oss << GetColorCode(msg._level) << FormatedMessage(msg) << "\033[m";
    return oss.str();
  }

  std::string GetColorCode(const LEVELS& level) const {
    if (level.value == WARNING.value) {
      return "\033[33m";  // yellow
    }
    if (level.value == DEBUG.value) {
      return "\033[32m";  // green
    }
    if (level.value == ERROR.value) {
      return "\033[31m";  // red
    }
    if (level.value == USER.value) {
      return "\033[1m\033[34m";  // bold blue
    }
    if (g3::internal::wasFatal(level)) {
      return "\033[1m\033[31m";  // red
    }
    return "\033[97m";  // white
  }
};

}  // namespace g3