#include "log.h"

namespace g3 {
void InitG3Logging(bool log_to_file, const std::string& prefix,
                   const std::string& path) {
  static std::unique_ptr<g3::LogWorker> worker;
  if (worker != nullptr) return;
  worker = std::move(g3::LogWorker::createLogWorker());
  worker->addSink(std::make_unique<g3::CustomSink>(),
                  &g3::CustomSink::StdLogMessage);
  if (log_to_file) {
    std::filesystem::create_directories(path);
    std::ostringstream oss;
    oss << path;
    if (*path.rbegin() != '/') oss << '/';
    if (prefix.empty()) oss << "g3log";
    oss << prefix << ".";
    auto now = std::chrono::system_clock::now();
    oss << g3::localtime_formatted(now, "%Y%m%d-%H%M%S");
    oss << ".log";
    worker->addSink(std::make_unique<g3::CustomSink>(oss.str()),
                    &g3::CustomSink::FileLogMessage);
  }
  g3::initializeLogging(worker.get());
}
}