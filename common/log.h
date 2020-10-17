#pragma once

#include <chrono>
#include <fstream>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <iostream>

const LEVELS ERROR{WARNING.value + 100, "ERROR"};
const LEVELS USER(ERROR.value + 100, "USER");

#define ADEBUG LOG(DEBUG)
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AUSER LOG(USER)
#define AFATAL LOG(FATAL)

// LOG_IF
#define ADEBUG_IF(cond) LOG_IF(DEBUG, cond)
#define AINFO_IF(cond) LOG_IF(INFO, cond)
#define AWARN_IF(cond) LOG_IF(WARNING, cond)
#define AERROR_IF(cond) LOG_IF(ERROR, cond)
#define AUSER_IF(cond) LOG_IF(USER, cond)
#define AFATAL_IF(cond) LOG_IF(FATAL, cond)
#define ACHECK(cond) CHECK(cond)

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

void InitG3Logging(bool log_to_file = false, const std::string& prefix = "",
                   const std::string& path = "./");

}  // namespace g3