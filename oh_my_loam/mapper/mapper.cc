#include "mapper.h"

#include <mutex>

#include "common/log/log.h"
#include "common/math/fitting.h"
#include "common/pcl/pcl_utils.h"

namespace oh_my_loam {

namespace {
using common::YAMLConfig;
using LineCoeff = Eigen::Matrix<double, 6, 1>;
}  // namespace

bool Mapper::Init() {
  const auto &config = YAMLConfig::Instance()->config();
  config_ = config["mapper_config"];
  is_vis_ = config["vis"].as<bool>() && config_["vis"].as<bool>();
  verbose_ = config_["vis"].as<bool>();
  AINFO << "Mapping visualizer: " << (is_vis_ ? "ON" : "OFF");
  map_shape_ = YAMLConfig::GetSeq<int>(config_["map_shape"]);
  submap_shape_ = YAMLConfig::GetSeq<int>(config_["submap_shape"]);
  map_step_ = config_["map_step"].as<double>();
  corn_map_.reset(new Map(map_shape_, map_step_));
  surf_map_.reset(new Map(map_shape_, map_step_));
  return true;
}

void Mapper::Reset() {
  SetState(UN_INIT);
}

void Mapper::Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
                     const TPointCloudConstPtr &cloud_surf,
                     const common::Pose3d &pose_curr2odom,
                     common::Pose3d *const pose_curr2map) {
  if (GetState() == UN_INIT) {
    pose_curr2map->SetIdentity();
    pose_odom2map_.SetIdentity();
    UpdateMap(*pose_curr2map, cloud_corn, cloud_surf);
    SetState(DONE);
    AINFO << "Mapper initialized...";
    return;
  }
  if (GetState() == DONE) {
    thread_.reset(new std::thread(&Mapper::Run, this, cloud_corn, cloud_surf,
                                  pose_curr2odom));
    if (thread_->joinable()) thread_->detach();
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  *pose_curr2map = pose_odom2map_ * pose_curr2odom;
}

void Mapper::Run(const TPointCloudConstPtr &cloud_corn,
                 const TPointCloudConstPtr &cloud_surf,
                 const common::Pose3d &pose_curr2odom) {
  BLOCK_TIMER_START;
  SetState(RUNNING);
  common::Pose3d pose_curr2map = pose_odom2map_ * pose_curr2odom;
  TPoint cnt(pose_curr2map.t_vec().x(), pose_curr2map.t_vec().y(),
             pose_curr2map.t_vec().z());
  // AdjustMap(cnt);
  TPointCloudPtr cloud_corn_map = corn_map_->GetSurrPoints(cnt, submap_shape_);
  TPointCloudPtr cloud_surf_map = surf_map_->GetSurrPoints(cnt, submap_shape_);
  for (int i = 0; i < config_["icp_iter_num"].as<int>(); ++i) {
    pcl::KdTreeFLANN<TPoint> kdtree_corn;
    kdtree_corn.setInputCloud(cloud_corn_map);
    pcl::KdTreeFLANN<TPoint> kdtree_surf;
    kdtree_surf.setInputCloud(cloud_surf_map);
    std::vector<PointLinePair> pl_pairs;
    MatchCorn(kdtree_corn, cloud_corn_map, pose_curr2map, &pl_pairs);
    std::vector<PointPlaneCoeffPair> pp_pairs;
    MatchSurf(kdtree_surf, cloud_surf_map, pose_curr2map, &pp_pairs);
    AINFO_IF(verbose_) << "Iter " << i
                       << ": matched num: point2line = " << pl_pairs.size()
                       << ", point2plane = " << pp_pairs.size();
    if (pl_pairs.size() + pp_pairs.size() <
        config_["min_correspondence_num"].as<size_t>()) {
      AWARN << "Too less correspondence: " << pl_pairs.size() << " + "
            << pp_pairs.size();
      continue;
    }
    PoseSolver solver(pose_curr2map);
    for (const auto &pair : pl_pairs) {
      solver.AddPointLinePair(pair, 1.0);
    }
    for (const auto &pair : pp_pairs) {
      solver.AddPointPlaneCoeffPair(pair, 1.0);
    }
    if (!solver.Solve(config_["solve_iter_num"].as<int>(), verbose_,
                      &pose_curr2map)) {
      AWARN << "Mapping solve: no_convergence";
    }
    AINFO_IF(verbose_) << "Odometer::ICP: iter_" << i << ": "
                       << BLOCK_TIMER_STOP_FMT;
  }
  UpdateMap(pose_curr2map, cloud_corn, cloud_surf);
  SetState(DONE);  // be careful with deadlock
  std::lock_guard<std::mutex> lock(state_mutex_);
  pose_odom2map_ = pose_curr2map * pose_curr2odom.Inv();
  AUSER << "Mapper::Run: " << BLOCK_TIMER_STOP_FMT;
}

void Mapper::MatchCorn(const pcl::KdTreeFLANN<TPoint> &kdtree,
                       const TPointCloudConstPtr &cloud_curr,
                       const common::Pose3d &pose_curr2map,
                       std::vector<PointLinePair> *const pairs) const {
  std::vector<int> indices;
  std::vector<float> dists;
  int nearest_neighbor_k = config_["nearest_neighbor_k"].as<int>();
  float neighbor_point_dist_sq_th =
      config_["neighbor_point_dist_sq_th"].as<float>();
  for (const auto &pt : *cloud_curr) {
    TPoint pt_query = common::TransformPoint(pose_curr2map, pt);
    if (kdtree.nearestKSearch(pt_query, nearest_neighbor_k, indices, dists) !=
        nearest_neighbor_k) {
      continue;
    }
    if (dists.back() > neighbor_point_dist_sq_th) continue;
    TPointCloud neighbor_pts;
    pcl::copyPointCloud(*cloud_curr, indices, neighbor_pts);
    double fit_score = 0.0;
    LineCoeff coeff = common::FitLine3D(neighbor_pts, &fit_score);
    if (fit_score < config_["min_line_fit_score"].as<double>()) continue;
    TPoint pt1(coeff[0], coeff[1], coeff[2], 0.0);
    TPoint pt2(coeff[0] + 0.1 * coeff[3], coeff[1] + 0.1 * coeff[4],
               coeff[2] + 0.1 * coeff[5], 0.0);
    pairs->emplace_back(pt, pt1, pt2);
  }
}

void Mapper::MatchSurf(const pcl::KdTreeFLANN<TPoint> &kdtree,
                       const TPointCloudConstPtr &cloud_curr,
                       const common::Pose3d &pose_curr2map,
                       std::vector<PointPlaneCoeffPair> *const pairs) const {
  std::vector<int> indices;
  std::vector<float> dists;
  int nearest_neighbor_k = config_["nearest_neighbor_k"].as<int>();
  float neighbor_point_dist_sq_th =
      config_["neighbor_point_dist_sq_th"].as<float>();
  for (const auto &pt : *cloud_curr) {
    TPoint pt_query = common::TransformPoint(pose_curr2map, pt);
    if (kdtree.nearestKSearch(pt_query, nearest_neighbor_k, indices, dists) !=
        nearest_neighbor_k) {
      continue;
    }
    if (dists.back() > neighbor_point_dist_sq_th) continue;
    TPointCloud neighbor_pts;
    pcl::copyPointCloud(*cloud_curr, indices, neighbor_pts);
    double fit_score = 0.0;
    Eigen::Vector4d coeff = common::FitPlane(neighbor_pts, &fit_score);
    if (fit_score < config_["min_plane_fit_score"].as<double>()) continue;
    TPoint pt1(coeff[0], coeff[1], coeff[2], 0.0);
    pairs->emplace_back(pt, coeff);
  }
}

void Mapper::AdjustMap(const TPoint &center) {
  Index index = corn_map_->GetIndex(center);
  int min_idx_z = submap_shape_[0] / 2 + 1;
  int max_idx_z = map_shape_[0] - min_idx_z - 1;
  if (index.k < min_idx_z) {
    corn_map_->ShiftZ(index.k - min_idx_z);
    surf_map_->ShiftZ(index.k - min_idx_z);
  }
  if (index.k > max_idx_z) {
    corn_map_->ShiftZ(index.k - max_idx_z);
    surf_map_->ShiftZ(index.k - max_idx_z);
  }
  int min_idx_y = submap_shape_[0] / 2 + 1;
  int max_idx_y = map_shape_[1] - min_idx_y - 1;
  if (index.j < min_idx_y) {
    corn_map_->ShiftY(index.j - min_idx_y);
    surf_map_->ShiftY(index.j - min_idx_y);
  }
  if (index.j > max_idx_y) {
    corn_map_->ShiftY(index.j - max_idx_y);
    surf_map_->ShiftY(index.j - max_idx_y);
  }
  int min_idx_x = submap_shape_[0] / 2 + 1;
  int max_idx_x = map_shape_[2] - min_idx_x - 1;
  if (index.i < min_idx_x) {
    corn_map_->ShiftX(index.i - min_idx_x);
    surf_map_->ShiftX(index.i - min_idx_x);
  }
  if (index.i > max_idx_x) {
    corn_map_->ShiftX(index.i - max_idx_x);
    surf_map_->ShiftX(index.i - max_idx_x);
  }
}

void Mapper::UpdateMap(const common::Pose3d &pose_curr2map,
                       const TPointCloudConstPtr &cloud_corn,
                       const TPointCloudConstPtr &cloud_surf) {
  auto update_map = [&](const TPointCloudConstPtr &cloud, Map *const map) {
    TPointCloudPtr cloud_trans(new TPointCloud);
    common::TransformPointCloud(pose_curr2map, *cloud, cloud_trans.get());
    std::vector<Index> indices;
    map->AddPoints(cloud_trans, &indices);
    map->Downsample(indices, config_["downsample_voxel_size"].as<double>());
  };
  std::lock_guard<std::mutex> lock(map_mutex_);
  update_map(cloud_corn, corn_map_.get());
  update_map(cloud_surf, surf_map_.get());
}

TPointCloudPtr Mapper::GetMapCloudCorn() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return corn_map_->GetAllPoints();
}

TPointCloudPtr Mapper::GetMapCloudSurf() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return surf_map_->GetAllPoints();
}

TPointCloudPtr Mapper::GetMapCloud() const {
  TPointCloudPtr map_points(new TPointCloud);
  std::lock_guard<std::mutex> lock(map_mutex_);
  *map_points += *corn_map_->GetAllPoints();
  *map_points += *surf_map_->GetAllPoints();
  return map_points;
}

Mapper::State Mapper::GetState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

void Mapper::SetState(State state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = state;
}

}  // namespace oh_my_loam