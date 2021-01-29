#include "mapper.h"

#include <mutex>

#include "common/log/log.h"
#include "common/pcl/pcl_utils.h"
#include "oh_my_loam/base/helper.h"
#include "oh_my_loam/base/types.h"
#include "oh_my_loam/mapper/map.h"
#include "oh_my_loam/solver/solver.h"

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
  corn_map_.reset(new Map(shape, config_["map_step"].as<double>()));
  surf_map_.reset(new Map(shape, config_["map_step"].as<double>()));
  return true;
}

void Mapper::Reset() {}

void Mapper::Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
                     const TPointCloudConstPtr &cloud_surf,
                     const common::Pose3d &pose_curr2odom,
                     common::Pose3d *const pose_curr2map) {
  if (GetState() == UN_INIT) {
    corn_map_->AddPoints(cloud_corn);
    surf_map_->AddPoints(cloud_surf);
    pose_curr2map->SetIdentity();
    pose_odom2map_.SetIdentity();
    SetState(DONE);
    return;
  }
  if (GetState() == DONE) {
    thread_.reset(new std::thread(&Mapper::Run, this, cloud_corn, cloud_surf,
                                  pose_curr2odom));
    thread_->detach();
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  *pose_curr2map = pose_odom2map_ * pose_curr2odom;
}

void Mapper::Run(const TPointCloudConstPtr &cloud_corn_in,
                 const TPointCloudConstPtr &cloud_surf_in,
                 const Pose3d &pose_curr2odom) {
  SetState(RUNNING);
  Pose3d pose_curr2map = pose_odom2map_ * pose_curr2odom;
  TPoint cnt(pose_curr2map.t_vec().x(), pose_curr2map.t_vec().y(),
             pose_curr2map.t_vec().z());
  AdjustMap(corn_map_->GetIndex(cnt));
  int submap_shape = config_["submap_shape"].as<int>();
  TPointCloudPtr cloud_corn(new TPointCloud);
  TPointCloudPtr cloud_surf(new TPointCloud);
  TransformPointCloud(pose_curr2map, *cloud_corn_in, cloud_corn.get());
  TransformPointCloud(pose_curr2map, *cloud_surf_in, cloud_surf.get());
  TPointCloudPtr cloud_corn_map =
      corn_map_->GetSurrPoints(cnt, submap_shape / 2);
  TPointCloudPtr cloud_surf_map =
      corn_map_->GetSurrPoints(cnt, submap_shape / 2);
  // MatchCorn();
  // MatchSurf();
  PoseSolver solver(pose_curr2map);
  if (!solver.Solve(5, false, &pose_curr2map)) {
    AWARN << "Solve: no_convergence";
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  pose_odom2map_ = pose_curr2map * pose_curr2odom.Inv();
  SetState(DONE);
}

void Mapper::AdjustMap(const Index &index) {
  std::vector<int> map_shape = corn_map_->Shape();
  int min_idx = config_["submap_shape"].as<int>() / 2 + 1;
  int max_idx_z = map_shape[0] - min_idx - 1;
  if (index.k < min_idx) {
    corn_map_->ShiftZ(index.k - min_idx);
    surf_map_->ShiftZ(index.k - min_idx);
  }
  if (index.k > max_idx_z) {
    corn_map_->ShiftZ(index.k - max_idx_z);
    surf_map_->ShiftZ(index.k - max_idx_z);
  }
  int max_idx_y = map_shape[1] - min_idx - 1;
  if (index.j < min_idx) {
    corn_map_->ShiftY(index.j - min_idx);
    surf_map_->ShiftY(index.j - min_idx);
  }
  if (index.j > max_idx_y) {
    corn_map_->ShiftY(index.j - max_idx_y);
    surf_map_->ShiftY(index.j - max_idx_y);
  }
  int max_idx_x = map_shape[2] - min_idx - 1;
  if (index.i < min_idx) {
    corn_map_->ShiftX(index.i - min_idx);
    surf_map_->ShiftX(index.i - min_idx);
  }
  if (index.i > max_idx_x) {
    corn_map_->ShiftX(index.i - max_idx_x);
    surf_map_->ShiftX(index.i - max_idx_x);
  }
}

void Mapper::Visualize() {}

}  // namespace oh_my_loam