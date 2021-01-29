#include "mapper.h"

#include <mutex>

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
  std::vector<int> shape = YAMLConfig::GetSeq<int>(config_["map_shape"]);
  cloud_corn_map_.reset(new Map(shape, config_["map_step"].as<double>()));
  cloud_surf_map_.reset(new Map(shape, config_["map_step"].as<double>()));
  return true;
}

void Mapper::Reset() {}

void Mapper::Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
                     const TPointCloudConstPtr &cloud_surf,
                     common::Pose3d *const pose_out) {
  if (GetState() == UN_INIT) {
    cloud_corn_map_->AddPoints(cloud_corn);
    cloud_surf_map_->AddPoints(cloud_surf);
    pose_out->SetIdentity();
    SetState(DONE);
    return;
  }
  if (GetState() == DONE) {
  } else {  // RUNNING
  }
}

void Mapper::Run(double timestamp, const TPointCloudConstPtr &cloud_corn,
                 const TPointCloudConstPtr &cloud_surf) {
  TimePose pose;
  pose.timestamp = timestamp;
  std::lock_guard<std::mutex> lock(mutex_);
}

void Mapper::Visualize() {}

}  // namespace oh_my_loam