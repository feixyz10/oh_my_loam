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

void Mapper::Reset() {}

void Mapper::Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
                     const TPointCloudConstPtr &cloud_surf,
                     common::Pose3d *const pose_out) {
  if (!is_initialized_) {
    cloud_corn_map_ = cloud_corn;
    cloud_surf_map_ = cloud_surf;
    pose_out->SetIdentity();
    is_initialized_ = true;
    return;
  }
}

void Mapper::Visualize() {}

}  // namespace oh_my_loam