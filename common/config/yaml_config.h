#pragma once

#include <yaml-cpp/yaml.h>

#include <string>

#include "common/log/log.h"
#include "common/macro/macros.h"

namespace common {

class YAMLConfig {
 public:
  void Init(const std::string &file);

  template <typename T>
  const T Get(const std::string &key) const {
    AFATAL_IF(!config_) << "Not initialized, please call Init first.";
    return (*config_)[key].as<T>();
  }

  const YAML::Node &config() const {
    AFATAL_IF(!config_) << "Not initialized, please call Init first.";
    return *config_;
  }

  template <typename T>
  static const std::vector<T> GetSeq(const YAML::Node &node) {
    ACHECK(node.IsSequence());
    std::vector<T> seq;
    for (auto it = node.begin(); it != node.end(); ++it) {
      seq.push_back(it->as<T>());
    }
    return seq;
  }

  template <typename TK, typename TV>
  static const std::map<TK, TV> GetMap(const YAML::Node &node) {
    ACHECK(node.IsMap());
    std::map<TK, TV> map;
    for (auto it = node.begin(); it != node.end(); ++it) {
      map.insert({it->first.as<TK>(), it->second.as<TV>()});
    }
    return map;
  }

 private:
  std::unique_ptr<YAML::Node> config_{nullptr};

  DECLARE_SINGLETON(YAMLConfig);
};

}  // namespace common