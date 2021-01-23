#include "odometer.h"

#include <algorithm>

#include "common/log/log.h"
#include "common/pcl/pcl_utils.h"
#include "common/time/timer.h"
#include "oh_my_loam/base/types.h"
#include "oh_my_loam/solver/solver.h"

namespace oh_my_loam {

namespace {
int kNearScanN = 2;
size_t kMinMatchNum = 10;
int GetScanId(const TPoint &point) {
  return static_cast<int>(point.time);
}
float GetTime(const TPoint &point) {
  return point.time - GetScanId(point);
}
}  // namespace

bool Odometer::Init() {
  const auto &config = common::YAMLConfig::Instance()->config();
  config_ = config["odometer_config"];
  is_vis_ = config["vis"].as<bool>() && config_["vis"].as<bool>();
  verbose_ = config_["verbose"].as<bool>();
  AINFO << "Odometry visualizer: " << (is_vis_ ? "ON" : "OFF");
  if (is_vis_) visualizer_.reset(new OdometerVisualizer);
  return true;
}

void Odometer::Process(double timestamp, const std::vector<Feature> &features,
                       Pose3d *const pose_out) {
  if (!is_initialized_) {
    UpdatePre(features);
    is_initialized_ = true;
    pose_curr2last_.SetIdentity();
    pose_curr2world_.SetIdentity();
    pose_out->SetIdentity();
    AWARN << "Odometer initialized....";
    return;
  }
  BLOCK_TIMER_START;
  AINFO << "Pose before: " << pose_curr2world_.ToString();
  std::vector<PointLinePair> pl_pairs;
  std::vector<PointPlanePair> pp_pairs;
  AINFO_IF(verbose_) << "Points to be matched: "
                     << kdtree_corn_.getInputCloud()->size() << " + "
                     << kdtree_surf_.getInputCloud()->size();
  for (int i = 0; i < config_["icp_iter_num"].as<int>(); ++i) {
    for (const auto &feature : features) {
      MatchCorn(*feature.cloud_sharp_corner, &pl_pairs);
      MatchSurf(*feature.cloud_flat_surf, &pp_pairs);
    }
    AINFO_IF(verbose_) << "Iter " << i
                       << ": matched num: point2line = " << pl_pairs.size()
                       << ", point2plane = " << pp_pairs.size();
    if (pl_pairs.size() + pp_pairs.size() < kMinMatchNum) {
      AWARN << "Too less correspondence: " << pl_pairs.size() << " + "
            << pp_pairs.size();
      continue;
    }
    PoseSolver solver(pose_curr2last_);
    for (const auto &pair : pl_pairs) {
      solver.AddPointLinePair(pair, pair.pt.time);
    }
    for (const auto &pair : pp_pairs) {
      solver.AddPointPlanePair(pair, pair.pt.time);
    }
    solver.Solve(config_["solve_iter_num"].as<int>(), verbose_,
                 &pose_curr2last_);
    AINFO << "Odometer::ICP: iter_" << i << ": " << BLOCK_TIMER_STOP_FMT;
  }
  pose_curr2world_ = pose_curr2world_ * pose_curr2last_;
  *pose_out = pose_curr2world_;
  AINFO << "Pose increase: " << pose_curr2last_.ToString();
  AINFO << "Pose after: " << pose_curr2world_.ToString();
  UpdatePre(features);
  AINFO << "Odometer::Porcess: " << BLOCK_TIMER_STOP_FMT;
  if (is_vis_) Visualize(features, pl_pairs, pp_pairs);
}

void Odometer::MatchCorn(const TPointCloud &src,
                         std::vector<PointLinePair> *const pairs) const {
  double dist_sq_thresh = config_["match_dist_sq_th"].as<double>();
  for (const auto &point : src) {
    TPoint query_pt = TransformToStart(pose_curr2last_, point);
    std::vector<int> indices(1);
    std::vector<float> dists(1);
    if (kdtree_corn_.nearestKSearch(query_pt, 1, indices, dists) != 1) {
      continue;
    }
    if (dists[0] >= dist_sq_thresh) continue;
    TPoint pt1 = kdtree_corn_.getInputCloud()->at(indices[0]);
    int pt1_scan_id = GetScanId(pt1);

    bool pt2_fount = false;
    float min_pt2_dist_squre = dist_sq_thresh;
    TPoint pt2;
    int i_begin = std::max<int>(0, pt1_scan_id - kNearScanN);
    int i_end =
        std::min<int>(kdtrees_scan_corn_.size(), pt1_scan_id + kNearScanN + 1);
    for (int i = i_begin; i < i_end && i != pt1_scan_id; ++i) {
      const auto &kdtree = kdtrees_scan_corn_[i];
      if (kdtree.nearestKSearch(query_pt, 1, indices, dists) != 0) {
        continue;
      }
      if (dists[0] < min_pt2_dist_squre) {
        pt2_fount = true;
        pt2 = kdtree.getInputCloud()->at(indices[0]);
        min_pt2_dist_squre = dists[0];
      }
    }
    if (!pt2_fount) continue;

    TPoint pt(point.x, point.y, point.z, GetTime(point));
    pt1.time = GetTime(pt1);
    pt2.time = GetTime(pt2);
    pairs->emplace_back(pt, pt1, pt2);
  }
}

void Odometer::MatchSurf(const TPointCloud &src,
                         std::vector<PointPlanePair> *const pairs) const {
  double dist_sq_thresh = config_["match_dist_sq_th"].as<double>();
  for (const auto &point : src) {
    TPoint query_pt = TransformToStart(pose_curr2last_, point);
    std::vector<int> indices;
    std::vector<float> dists;
    if (kdtree_surf_.nearestKSearch(query_pt, 2, indices, dists) != 2) {
      continue;
    }
    if (dists[0] >= dist_sq_thresh || dists[1] >= dist_sq_thresh) continue;
    TPoint pt1 = kdtree_surf_.getInputCloud()->at(indices[0]);
    int pt1_scan_id = GetScanId(pt1);

    TPoint pt2 = kdtree_surf_.getInputCloud()->at(indices[1]), pt3;
    bool pt2_found = true, pt3_found = false;
    int pt2_scan_id = GetScanId(pt2);
    if (pt2_scan_id != pt1_scan_id) {
      pt2_found = false;
      if (std::abs(pt2_scan_id - pt1_scan_id) <= kNearScanN) {
        pt3 = pt2;
        pt3_found = true;
      }
    }

    if (!pt2_found) {
      const auto &kdtree = kdtrees_scan_surf_[pt1_scan_id];
      if (kdtree.nearestKSearch(query_pt, 2, indices, dists) == 2) {
        if (dists[1] < dist_sq_thresh) {
          pt2 = kdtree.getInputCloud()->at(indices[1]);
          pt2_found = true;
        }
      }
    }
    if (!pt2_found) continue;

    if (!pt3_found) {
      float min_pt3_dist_squre = dist_sq_thresh;
      int i_begin = std::max<int>(0, pt1_scan_id - kNearScanN);
      int i_end = std::min<int>(kdtrees_scan_corn_.size(),
                                pt1_scan_id + kNearScanN + 1);
      for (int i = i_begin; i < i_end && i != pt1_scan_id; ++i) {
        const auto &kdtree = kdtrees_scan_surf_[i];
        if (kdtree.nearestKSearch(query_pt, 1, indices, dists) != 1) {
          continue;
        }
        if (dists[0] < min_pt3_dist_squre) {
          pt3 = kdtree.getInputCloud()->at(indices[0]);
          pt3_found = true;
          min_pt3_dist_squre = dists[0];
        }
      }
    }
    if (!pt3_found) continue;

    TPoint pt(point.x, point.y, point.z, GetTime(point));
    pt1.time = GetTime(pt1);
    pt2.time = GetTime(pt2);
    pt3.time = GetTime(pt3);
    pairs->emplace_back(pt, pt1, pt2, pt3);
  }
}

void Odometer::UpdatePre(const std::vector<Feature> &features) {
  kdtrees_scan_corn_.resize(features.size());
  kdtrees_scan_surf_.resize(features.size());
  TPointCloudPtr corn_pre(new TPointCloud);
  TPointCloudPtr surf_pre(new TPointCloud);
  TPointCloudPtr scan_corn_pre(new TPointCloud);
  TPointCloudPtr scan_surf_pre(new TPointCloud);
  for (size_t i = 0; i < features.size(); ++i) {
    const auto &feature = features[i];
    scan_corn_pre->resize(feature.cloud_corner->size());
    scan_surf_pre->resize(feature.cloud_surf->size());
    for (size_t j = 0; j < feature.cloud_corner->size(); ++j) {
      TransformToEnd(pose_curr2last_, feature.cloud_corner->at(j),
                     &scan_corn_pre->at(j));
    }
    for (size_t j = 0; j < feature.cloud_surf->size(); ++j) {
      TransformToEnd(pose_curr2last_, feature.cloud_surf->at(j),
                     &scan_surf_pre->at(j));
    }
    *corn_pre += *scan_corn_pre;
    *surf_pre += *scan_surf_pre;
    kdtrees_scan_corn_[i].setInputCloud(scan_corn_pre);
    kdtrees_scan_surf_[i].setInputCloud(scan_surf_pre);
  }
  kdtree_corn_.setInputCloud(corn_pre);
  kdtree_surf_.setInputCloud(surf_pre);
}

void Odometer::Visualize(const std::vector<Feature> &features,
                         const std::vector<PointLinePair> &pl_pairs,
                         const std::vector<PointPlanePair> &pp_pairs,
                         double timestamp) const {
  std::shared_ptr<OdometerVisFrame> frame(new OdometerVisFrame);
  frame->timestamp = timestamp;
  frame->pl_pairs = pl_pairs;
  frame->pp_pairs = pp_pairs;
  frame->pose_curr2last = pose_curr2last_;
  frame->pose_curr2world = pose_curr2world_;
  visualizer_->Render(frame);
}
}  // namespace oh_my_loam
