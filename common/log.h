#pragma once

#include <chrono>
#include <fstream>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <iostream>

const LEVELS ERROR{WARNING.value + 100, "ERROR"};

#define ADEBUG LOG(DEBUG) << "[DEBUG] "
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

// LOG_IF
#define AINFO_IF(cond) LOG_IF(INFO, cond)
#define AWARN_IF(cond) LOG_IF(WARNING, cond)
#define AERROR_IF(cond) LOG_IF(ERROR, cond)
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
    if (log_to_file_ && ofs_ != nullptr) {
      (*ofs_) << oss.str() << std::endl;
    }
    std::clog << oss.str() << std::endl;
  };

  void StdLogMessage(g3::LogMessageMover logEntry) {
    std::clog << ColorFormatedMessage(logEntry.get()) << std::endl;
  }

  void FileLogMessage(g3::LogMessageMover logEntry) {
    if (log_to_file_ && ofs_ != nullptr) {
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
    oss << "\033[" << GetColor(msg._level) << "m" << FormatedMessage(msg)
        << "\033[m";
    return oss.str();
  }

  int GetColor(const LEVELS& level) const {
    if (level.value == WARNING.value) {
      return 33;  // yellow
    }
    if (level.value == DEBUG.value) {
      return 32;  // green
    }
    if (level.value == ERROR.value) {
      return 31;  // red
    }
    if (g3::internal::wasFatal(level)) {
      return 31;  // red
    }
    return 97;  // white
  }
};

void InitG3Logging(bool log_to_file = false, const std::string& prefix = "",
                   const std::string& path = "./");

}  // namespace g3