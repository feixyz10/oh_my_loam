
#include "common/config/yaml_config.h"

namespace common {

YAMLConfig::YAMLConfig() {}

void YAMLConfig::Init(const std::string &file) {
  config_.reset(new YAML::Node);
  *config_ = YAML::LoadFile(file);
}

}  // namespace common