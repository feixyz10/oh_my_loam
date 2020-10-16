#pragma once

#include <yaml-cpp/yaml.h>

#include <string>

#include "log.h"
#include "macros.h"

namespace oh_my_loam {

class Config {
 public:
  void SetConfigFile(const std::string& file) {
    config_.reset(new YAML::Node);
    *config_ = YAML::LoadFile(file);
  }

  template <typename T>
  const T Get(const std::string& key) const {
    AFATAL_IF(!config_) << "No config exists: please call SetConfigFile.";
    return (*config_)[key].as<T>();
  }

  const YAML::Node& config() const {
    AFATAL_IF(!config_) << "No config exists: please call SetConfigFile.";
    return *config_;
  }

 private:
  std::shared_ptr<YAML::Node> config_{nullptr};

  DECLARE_SINGLETON(Config)
};

}  // namespace oh_my_loam