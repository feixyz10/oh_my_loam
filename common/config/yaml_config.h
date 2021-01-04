#pragma once

#include <yaml-cpp/yaml.h>
#include <string>

#include "common/log/log.h"
#include "common/macro/macros.h"

namespace common {

class YAMLConfig {
 public:
  void Init(const std::string& file) {
    config_.reset(new YAML::Node);
    *config_ = YAML::LoadFile(file);
  }

  template <typename T>
  const T Get(const std::string& key) const {
    AFATAL_IF(!config_) << "Not initialized, please call Init first.";
    return (*config_)[key].as<T>();
  }

  const YAML::Node& config() const {
    AFATAL_IF(!config_) << "Not initialized, please call Init first.";
    return *config_;
  }

 private:
  std::unique_ptr<YAML::Node> config_{nullptr};
  DECLARE_SINGLETON(YAMLConfig);
};

}  // namespace common