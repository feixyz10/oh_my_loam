#include "odometer.h"

#include "common/pcl/pcl_utils.h"
#include "oh_my_loam/solver/solver.h"

namespace oh_my_loam {

namespace {
int kNearScanN = 2;
size_t kMinMatchNum = 10;
using namespace common;
}  // namespace

bool Odometer::Init() {
  const auto &config = YAMLConfig::Instance()->config();
  config_ = config["odometer_config"];
  is_vis_ = config["vis"].as<bool>() && config_["vis"].as<bool>();
  verbose_ = config_["verbose"].as<bool>();
  AINFO << "Odometry visualizer: " << (is_vis_ ? "ON" : "OFF");
  if (is_vis_) visualizer_.reset(new OdometerVisualizer);
  return true;
}

void Odometer::Process(double timestamp, const std::vector<Feature> &features,
                       Pose3d *const pose_out) {
  BLOCK_TIMER_START;
  if (!is_initialized_) {
    UpdatePre(features);
    is_initialized_ = true;
    pose_curr2last_.SetIdentity();
    pose_curr2world_.SetIdentity();
    pose_out->SetIdentity();
    AWARN << "Odometer initialized....";
    return;
  }
  AINFO_IF(verbose_) << "Pose before: " << pose_curr2world_.ToString();
  std::vector<PointLinePair> pl_pairs;
  std::vector<PointPlanePair> pp_pairs;
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
    solver.Solve(5, verbose_, &pose_curr2last_);
  }
  pose_curr2world_ = pose_curr2world_ * pose_curr2last_;
  *pose_out = pose_curr2world_;
  AINFO_IF(verbose_) << "Pose increase: " << pose_curr2last_.ToString();
  AINFO_IF(verbose_) << "Pose after: " << pose_curr2world_.ToString();
  UpdatePre(features);
  if (is_vis_) Visualize(features, pl_pairs, pp_pairs);
  AINFO << "Odometer::Porcess: " << BLOCK_TIMER_STOP_FMT;
}

void Odometer::MatchCorn(const TPointCloud &src,
                         std::vector<PointLinePair> *const pairs) const {
  double dist_sq_thresh = config_["match_dist_sq_th"].as<double>();
  auto comp = [](const std::vector<float> &v1, const std::vector<float> &v2) {
    return v1[0] < v2[0];
  };
  for (const auto &pt : src) {
    TPoint query_pt = TransformToStart(pose_curr2last_, pt);
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    NearestKSearch(kdtrees_corn_, query_pt, 1, &indices, &dists);
    int pt1_scan_id =
        std::min_element(dists.begin(), dists.end(), comp) - dists.begin();
    if (dists[pt1_scan_id][0] >= dist_sq_thresh) continue;
    TPoint pt1 = clouds_corn_pre_[pt1_scan_id]->at(indices[pt1_scan_id][0]);

    double min_dist_pt2_squre = dist_sq_thresh;
    int pt2_scan_id = -1;
    int i_begin = std::max<int>(0, pt1_scan_id - kNearScanN);
    int i_end = std::min<int>(indices.size(), pt1_scan_id + kNearScanN + 1);
    for (int i = i_begin; i < i_end && i != pt1_scan_id; ++i) {
      if (dists[i][0] < min_dist_pt2_squre) {
        pt2_scan_id = i;
        min_dist_pt2_squre = dists[i][0];
      }
    }
    if (pt2_scan_id >= 0) {
      TPoint pt2 = clouds_corn_pre_[pt2_scan_id]->at(indices[pt2_scan_id][0]);
      pairs->emplace_back(pt, pt1, pt2);
    }
  }
}

void Odometer::MatchSurf(const TPointCloud &src,
                         std::vector<PointPlanePair> *const pairs) const {
  double dist_sq_thresh = config_["match_dist_sq_th"].as<double>();
  auto comp = [](const std::vector<float> &v1, const std::vector<float> &v2) {
    return v1[0] < v2[0];
  };
  for (const auto &pt : src) {
    TPoint query_pt = TransformToStart(pose_curr2last_, pt);
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    NearestKSearch(kdtrees_surf_, query_pt, 2, &indices, &dists);
    int pt1_scan_id =
        std::min_element(dists.begin(), dists.end(), comp) - dists.begin();
    if (dists[pt1_scan_id][0] >= dist_sq_thresh) continue;
    if (dists[pt1_scan_id][1] >= dist_sq_thresh) continue;
    TPoint pt1 = clouds_surf_pre_[pt1_scan_id]->at(indices[pt1_scan_id][0]);
    TPoint pt2 = clouds_surf_pre_[pt1_scan_id]->at(indices[pt1_scan_id][1]);

    double min_dist_pt3_squre = dist_sq_thresh;
    int pt3_scan_id = -1;
    int i_begin = std::max<int>(0, pt1_scan_id - kNearScanN);
    int i_end = std::min<int>(indices.size(), pt1_scan_id + kNearScanN + 1);
    for (int i = i_begin; i < i_end && i != pt1_scan_id; ++i) {
      if (dists[i][0] < min_dist_pt3_squre) {
        pt3_scan_id = i;
        min_dist_pt3_squre = dists[i][0];
      }
    }
    if (pt3_scan_id >= 0) {
      TPoint pt3 = clouds_surf_pre_[pt3_scan_id]->at(indices[pt3_scan_id][0]);
      pairs->emplace_back(pt, pt1, pt2, pt3);
    }
  }
}

void Odometer::UpdatePre(const std::vector<Feature> &features) {
  BLOCK_TIMER_START;
  clouds_corn_pre_.resize(features.size());
  clouds_surf_pre_.resize(features.size());
  kdtrees_corn_.resize(features.size());
  kdtrees_surf_.resize(features.size());

  for (size_t i = 0; i < features.size(); ++i) {
    const auto &feature = features[i];
    auto &cloud_pre = clouds_corn_pre_[i];
    if (!cloud_pre) cloud_pre.reset(new TPointCloud);
    cloud_pre->resize(feature.cloud_corner->size());
    for (size_t j = 0; j < feature.cloud_corner->size(); ++j) {
      TransformToEnd(pose_curr2last_, feature.cloud_corner->at(j),
                     &cloud_pre->at(j));
    }
    kdtrees_corn_[i].setInputCloud(cloud_pre);
  }

  for (size_t i = 0; i < features.size(); ++i) {
    const auto &feature = features[i];
    auto &cloud_pre = clouds_surf_pre_[i];
    if (!cloud_pre) cloud_pre.reset(new TPointCloud);
    cloud_pre->resize(feature.cloud_surf->size());
    for (size_t j = 0; j < feature.cloud_surf->size(); ++j) {
      TransformToEnd(pose_curr2last_, feature.cloud_surf->at(j),
                     &cloud_pre->at(j));
    }
    kdtrees_surf_[i].setInputCloud(cloud_pre);
  }
  AINFO << "Odometer::UpdatePre: " << BLOCK_TIMER_STOP_FMT;
}

void Odometer::NearestKSearch(
    const std::vector<pcl::KdTreeFLANN<TPoint>> &kdtrees,
    const TPoint &query_pt, int k, std::vector<std::vector<int>> *const indices,
    std::vector<std::vector<float>> *const dists) const {
  for (const auto &kdtree : kdtrees) {
    std::vector<int> index;
    std::vector<float> dist;
    int n_found = 0;
    if (kdtree.getInputCloud()->size() >= static_cast<size_t>(k)) {
      n_found = kdtree.nearestKSearch(query_pt, k, index, dist);
    }
    if (n_found < k) {
      std::vector<int>(k, -1).swap(index);
      std::vector<float>(k, std::numeric_limits<float>::max()).swap(dist);
    }
    indices->push_back(std::move(index));
    dists->push_back(std::move(dist));
  }
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
