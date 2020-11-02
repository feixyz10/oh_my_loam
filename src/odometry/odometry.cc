#include "odometry.h"

#include "solver/solver.h"

namespace oh_my_loam {

namespace {
int kNearbyScanNum = 2;
size_t kMinMatchNum = 10;
}  // namespace

bool Odometry::Init(const YAML::Node& config) {
  config_ = config;
  kdtree_surf_pts_.reset(new pcl::KdTreeFLANN<TPoint>);
  kdtree_corn_pts_.reset(new pcl::KdTreeFLANN<TPoint>);
  return true;
}

void Odometry::Process(const FeaturePoints& feature, Pose3D* const pose) {
  if (!is_initialized) {
    is_initialized = true;
    UpdatePre(feature);
    *pose = pose_curr2world_;
    AWARN << "Odometry initialized....";
    return;
  }
  TicToc timer_whole;
  double match_dist_sq_thresh = config_["match_dist_sq_thresh"].as<double>();
  AINFO << "Pose before: " << pose_curr2world_.ToString();
  for (int i = 0; i < config_["icp_iter_num"].as<int>(); ++i) {
    TicToc timer;
    std::vector<PointLinePair> pl_pairs;
    std::vector<PointPlanePair> pp_pairs;
    MatchCornPoints(*feature.sharp_corner_pts, *corn_pts_pre_, &pl_pairs,
                    match_dist_sq_thresh);
    double time_pl_match = timer.toc();
    AINFO_IF(i == 0) << "PL mathch: src size = "
                     << feature.sharp_corner_pts->size()
                     << ", tgt size = " << corn_pts_pre_->size();
    MatchSurfPoints(*feature.flat_surf_pts, *surf_pts_pre_, &pp_pairs,
                    match_dist_sq_thresh);
    double timer_pp_match = timer.toc();
    AINFO_IF(i == 0) << "PP mathch: src size = "
                     << feature.flat_surf_pts->size()
                     << ", tgt size = " << surf_pts_pre_->size();
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
    pose_curr2last_ = Pose3D(q, p);
    double time_solve = timer.toc();
    AINFO << "Time elapsed (ms), pl_match = " << time_pl_match
          << ", pp_match = " << timer_pp_match - time_pl_match
          << ", solve = " << time_solve - timer_pp_match;
  }
  pose_curr2world_ = pose_curr2world_ * pose_curr2last_;
  *pose = pose_curr2world_;
  AWARN << "Pose increase: " << pose_curr2last_.ToString();
  AWARN << "Pose after: " << pose_curr2world_.ToString();
  UpdatePre(feature);
  AUSER << "Time elapsed (ms): whole odometry = " << timer_whole.toc();
}

void Odometry::MatchCornPoints(const TPointCloud& src, const TPointCloud& tgt,
                               std::vector<PointLinePair>* const pairs,
                               double dist_sq_thresh) const {
  for (const auto& q_pt : src) {
    TPoint query_pt;
    TransformToStart(pose_curr2last_, q_pt, &query_pt);
    std::vector<int> indices;
    std::vector<float> dists;
    kdtree_corn_pts_->nearestKSearch(query_pt, 1, indices, dists);
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

void Odometry::MatchSurfPoints(const TPointCloud& src, const TPointCloud& tgt,
                               std::vector<PointPlanePair>* const pairs,
                               double dist_sq_thresh) const {
  for (const auto& q_pt : src) {
    TPoint query_pt;
    TransformToStart(pose_curr2last_, q_pt, &query_pt);
    std::vector<int> indices;
    std::vector<float> dists;
    kdtree_surf_pts_->nearestKSearch(query_pt, 1, indices, dists);
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

void Odometry::UpdatePre(const FeaturePoints& feature) {
  if (surf_pts_pre_ == nullptr) {
    surf_pts_pre_.reset(new TPointCloud);
  }
  if (corn_pts_pre_ == nullptr) {
    corn_pts_pre_.reset(new TPointCloud);
  }

  surf_pts_pre_->resize(feature.less_flat_surf_pts->size());
  for (size_t i = 0; i < feature.less_flat_surf_pts->size(); ++i) {
    TransformToEnd(pose_curr2last_, feature.less_flat_surf_pts->points[i],
                   &surf_pts_pre_->points[i]);
  }
  corn_pts_pre_->resize(feature.less_sharp_corner_pts->size());
  for (size_t i = 0; i < feature.less_sharp_corner_pts->size(); ++i) {
    TransformToEnd(pose_curr2last_, feature.less_sharp_corner_pts->points[i],
                   &corn_pts_pre_->points[i]);
  }
  kdtree_surf_pts_->setInputCloud(surf_pts_pre_);
  kdtree_corn_pts_->setInputCloud(corn_pts_pre_);
}

}  // namespace oh_my_loam
