#include "mapper.h"

namespace oh_my_loam {

bool Mapper::Init(const YAML::Node& config) {
  config_ = config;
  is_vis_ = Config::Instance()->Get<bool>("vis") && config_["vis"].as<bool>();
  AINFO << "Mapping visualizer: " << (is_vis_ ? "ON" : "OFF");
  return true;
}

void Mapper::Process() {}

void Mapper::Visualize() {}

}  // namespace oh_my_loam