#include "mapper.h"

namespace oh_my_loam {

namespace {
using namespace common;
}  // namespace

bool Mapper::Init() {
  const auto &config = YAMLConfig::Instance()->config();
  config_ = config["mapper_config"];
  is_vis_ = config["vis"].as<bool>() && config_["vis"].as<bool>();
  verbose_ = config_["vis"].as<bool>();
  AINFO << "Mapping visualizer: " << (is_vis_ ? "ON" : "OFF");
  return true;
}

void Mapper::Process(double timestamp, const std::vector<Feature> &features,
                     Pose3d *const pose_out) {}

void Mapper::Visualize() {}

}  // namespace oh_my_loam