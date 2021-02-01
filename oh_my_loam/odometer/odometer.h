#pragma once

#include "common/common.h"
#include "oh_my_loam/base/feature.h"
#include "oh_my_loam/base/utils.h"
#include "oh_my_loam/solver/solver.h"
#include "oh_my_loam/visualizer/odometer_visualizer.h"

namespace oh_my_loam {

// frame to frame lidar Odometer
class Odometer {
 public:
  Odometer() = default;

  bool Init();

  void Process(double timestamp, const std::vector<Feature> &features,
               common::Pose3d *const pose_out);

  TPointCloudConstPtr cloud_corn() const {
    return kdtree_corn_.getInputCloud();
  }

  TPointCloudConstPtr cloud_surf() const {
    return kdtree_surf_.getInputCloud();
  }

  void Reset();

 protected:
  void UpdatePre(const std::vector<Feature> &features);

  void MatchCorn(const TPointCloud &src,
                 std::vector<PointLinePair> *const pairs) const;

  void MatchSurf(const TPointCloud &src,
                 std::vector<PointPlanePair> *const pairs) const;

  void Visualize(const std::vector<PointLinePair> &pl_pairs,
                 const std::vector<PointPlanePair> &pp_pairs,
                 double timestamp = 0.0) const;

  bool NearestSearch(const pcl::KdTreeFLANN<TPoint> &kdtree,
                     const TPoint &query_pt, int k, float dist_sq_th,
                     std::vector<int> *const indices) const;

  common::Pose3d pose_curr2world_;
  common::Pose3d pose_curr2last_;

  std::vector<pcl::KdTreeFLANN<TPoint>> kdtrees_scan_surf_;
  std::vector<pcl::KdTreeFLANN<TPoint>> kdtrees_scan_corn_;
  pcl::KdTreeFLANN<TPoint> kdtree_corn_;
  pcl::KdTreeFLANN<TPoint> kdtree_surf_;

  YAML::Node config_;

  bool is_initialized_ = false;

  bool is_vis_ = false;

  bool verbose_ = false;

  std::unique_ptr<OdometerVisualizer> visualizer_{nullptr};

  DISALLOW_COPY_AND_ASSIGN(Odometer)
};

}  // namespace oh_my_loam
