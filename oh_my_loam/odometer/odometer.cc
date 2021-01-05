#include "odometer.h"

#include "common/pcl/pcl_utils.h"
#include "oh_my_loam/solver/solver.h"

namespace oh_my_loam {

namespace {
int kNearbyScanNum = 2;
size_t kMinMatchNum = 10;
using namespace common;
}  // namespace

bool Odometer::Init() {
  const auto& config = YAMLConfig::Instance()->config();
  config_ = config["odometer_config"];
  is_vis_ = config["vis"].as<bool>() && config_["vis"].as<bool>();
  verbose_ = config_["verbose"].as<bool>();
  kdtree_surf_.reset(new pcl::KdTreeFLANN<TPoint>);
  kdtree_corn_.reset(new pcl::KdTreeFLANN<TPoint>);
  AINFO << "Odometer visualizer: " << (is_vis_ ? "ON" : "OFF");
  if (is_vis_) visualizer_.reset(new OdometerVisualizer);
  return true;
}

void Odometer::Process(const Feature& feature, Pose3d* const pose) {
  BLOCK_TIMER_START;
  if (!is_initialized_) {
    is_initialized_ = true;
    UpdatePre(feature);
    *pose = Pose3d();
    AWARN << "Odometer initialized....";
    return;
  }
  AINFO << "Pose before: " << pose_curr2world_.ToString();
  std::vector<PointLinePair> pl_pairs;
  std::vector<PointPlanePair> pp_pairs;
  for (int i = 0; i < config_["icp_iter_num"].as<int>(); ++i) {
    MatchCornFeature(*feature.cloud_sharp_corner, *corn_pre_, &pl_pairs);
    AINFO_IF(i == 0) << "PL mathch: src size = "
                     << feature.cloud_sharp_corner->size()
                     << ", tgt size = " << corn_pre_->size();
    MatchSurfFeature(*feature.cloud_flat_surf, *surf_pre_, &pp_pairs);
    AINFO_IF(i == 0) << "PP mathch: src size = "
                     << feature.cloud_flat_surf->size()
                     << ", tgt size = " << surf_pre_->size();
    AINFO << "Matched num, pl = " << pl_pairs.size()
          << ", pp = " << pp_pairs.size();
    if (pl_pairs.size() + pp_pairs.size() < kMinMatchNum) {
      AWARN << "Too less correspondence: num = "
            << pl_pairs.size() + pp_pairs.size();
    }
    Eigen::Vector4d q_vec = pose_curr2last_.q().coeffs();
    Eigen::Vector3d p_vec = pose_curr2last_.p();
    double* q = q_vec.data();
    double* p = p_vec.data();
    PoseSolver solver(q, p);
    for (const auto& pair : pl_pairs) {
      solver.AddPointLinePair(pair, GetTime(pair.pt));
    }
    for (const auto& pair : pp_pairs) {
      solver.AddPointPlanePair(pair, GetTime(pair.pt));
    }
    solver.Solve();
    pose_curr2last_ = Pose3d(q, p);
  }
  pose_curr2world_ = pose_curr2world_ * pose_curr2last_;
  *pose = pose_curr2world_;
  AWARN << "Pose increase: " << pose_curr2last_.ToString();
  AWARN << "Pose after: " << pose_curr2world_.ToString();
  UpdatePre(feature);
  AINFO << "Odometer::Porcess: " << BLOCK_TIMER_STOP_FMT;
  if (is_vis_) Visualize(feature, pl_pairs, pp_pairs);
}

void Odometer::MatchCornFeature(const TPointCloud& src, const TPointCloud& tgt,
                                std::vector<PointLinePair>* const pairs) const {
  double dist_sq_thresh = config_["match_dist_sq_thresh"].as<double>();
  for (const auto& q_pt : src) {
    TPoint query_pt;
    TransformToStart(pose_curr2last_, q_pt, &query_pt);
    std::vector<int> indices;
    std::vector<float> dists;
    kdtree_corn_->nearestKSearch(query_pt, 1, indices, dists);
    if (dists[0] >= dist_sq_thresh) continue;
    TPoint pt1 = tgt.points[indices[0]];
    int pt2_idx = -1;
    double min_dist_pt2_squre = dist_sq_thresh;
    int query_pt_scan_id = GetScanId(query_pt);
    for (int i = indices[0] + 1; i < static_cast<int>(tgt.size()); ++i) {
      const auto pt = tgt.points[i];
      int scan_id = GetScanId(pt);
      if (scan_id <= query_pt_scan_id) continue;
      if (scan_id > query_pt_scan_id + kNearbyScanNum) break;
      double dist_squre = DistanceSqure(query_pt, pt);
      if (dist_squre < min_dist_pt2_squre) {
        pt2_idx = i;
        min_dist_pt2_squre = dist_squre;
      }
    }
    for (int i = indices[0] - 1; i >= 0; --i) {
      const auto pt = tgt.points[i];
      int scan_id = GetScanId(pt);
      if (scan_id >= query_pt_scan_id) continue;
      if (scan_id < query_pt_scan_id - kNearbyScanNum) break;
      double dist_squre = DistanceSqure(query_pt, pt);
      if (dist_squre < min_dist_pt2_squre) {
        pt2_idx = i;
        min_dist_pt2_squre = dist_squre;
      }
    }
    if (pt2_idx >= 0) {
      TPoint pt2 = tgt.points[pt2_idx];
      pairs->emplace_back(q_pt, pt1, pt2);
    }
  }
}

void Odometer::MatchSurfFeature(
    const TPointCloud& src, const TPointCloud& tgt,
    std::vector<PointPlanePair>* const pairs) const {
  double dist_sq_thresh = config_["match_dist_sq_thresh"].as<double>();
  for (const auto& q_pt : src) {
    TPoint query_pt;
    TransformToStart(pose_curr2last_, q_pt, &query_pt);
    std::vector<int> indices;
    std::vector<float> dists;
    kdtree_surf_->nearestKSearch(query_pt, 1, indices, dists);
    if (dists[0] >= dist_sq_thresh) continue;
    TPoint pt1 = tgt.points[indices[0]];
    int pt2_idx = -1, pt3_idx = -1;
    double min_dist_pt2_squre = dist_sq_thresh;
    double min_dist_pt3_squre = dist_sq_thresh;
    int query_pt_scan_id = GetScanId(query_pt);
    for (int i = indices[0] + 1; i < static_cast<int>(tgt.size()); ++i) {
      const auto pt = tgt.points[i];
      int scan_id = GetScanId(pt);
      if (scan_id > query_pt_scan_id + kNearbyScanNum) break;
      double dist_squre = DistanceSqure(query_pt, pt);
      if (scan_id == query_pt_scan_id && dist_squre < min_dist_pt2_squre) {
        pt2_idx = i;
        min_dist_pt2_squre = dist_squre;
      }
      if (scan_id > query_pt_scan_id && dist_squre < min_dist_pt3_squre) {
        pt3_idx = i;
        min_dist_pt3_squre = dist_squre;
      }
      if (pt2_idx == -1 && scan_id > query_pt_scan_id) break;
    }
    for (int i = indices[0] - 1; i >= 0; --i) {
      const auto pt = tgt.points[i];
      int scan_id = GetScanId(pt);
      if (scan_id < query_pt_scan_id - kNearbyScanNum) break;
      double dist_squre = DistanceSqure(query_pt, pt);
      if (scan_id == query_pt_scan_id && dist_squre < min_dist_pt2_squre) {
        pt2_idx = i;
        min_dist_pt2_squre = dist_squre;
      }
      if (scan_id < query_pt_scan_id && dist_squre < min_dist_pt3_squre) {
        pt3_idx = i;
        min_dist_pt3_squre = dist_squre;
      }
      if (pt2_idx == -1 && scan_id < query_pt_scan_id) break;
    }
    if (pt2_idx >= 0 && pt3_idx >= 0) {
      TPoint pt2 = tgt.points[pt2_idx], pt3 = tgt.points[pt3_idx];
      pairs->emplace_back(q_pt, pt1, pt2, pt3);
    }
  }
}

void Odometer::UpdatePre(const Feature& feature) {
  if (!surf_pre_) {
    surf_pre_.reset(new TPointCloud);
  }
  if (!corn_pre_) {
    corn_pre_.reset(new TPointCloud);
  }

  surf_pre_->resize(feature.cloud_less_flat_surf->size());
  for (size_t i = 0; i < feature.cloud_less_flat_surf->size(); ++i) {
    TransformToEnd(pose_curr2last_, feature.cloud_less_flat_surf->points[i],
                   &surf_pre_->points[i]);
  }
  corn_pre_->resize(feature.cloud_less_sharp_corner->size());
  for (size_t i = 0; i < feature.cloud_less_sharp_corner->size(); ++i) {
    TransformToEnd(pose_curr2last_, feature.cloud_less_sharp_corner->points[i],
                   &corn_pre_->points[i]);
  }
  kdtree_surf_->setInputCloud(surf_pre_);
  kdtree_corn_->setInputCloud(corn_pre_);
}

void Odometer::Visualize(const Feature& feature,
                         const std::vector<PointLinePair>& pl_pairs,
                         const std::vector<PointPlanePair>& pp_pairs) const {
  std::shared_ptr<OdometerVisFrame> frame(new OdometerVisFrame);
  frame->pl_pairs = pl_pairs;
  frame->pp_pairs = pp_pairs;
  frame->pose_curr2last = pose_curr2last_;
  frame->pose_curr2world = pose_curr2world_;
  visualizer_->Render(frame);
}

}  // namespace oh_my_loam
