#pragma once

#include "extractor/feature_points.h"
#include "visualizer/odometry_visualizer.h"

#include "helper/helper.h"

namespace oh_my_loam {

// frame to frame lidar odometry
class Odometry {
 public:
  Odometry() = default;

  bool Init(const YAML::Node& config);

  void Process(const FeaturePoints& feature, Pose3d* const pose);

 protected:
  void UpdatePre(const FeaturePoints& feature);

  void MatchCornPoints(const TPointCloud& src, const TPointCloud& tgt,
                       std::vector<PointLinePair>* const pairs,
                       double dist_sq_thresh) const;

  void MatchSurfPoints(const TPointCloud& src, const TPointCloud& tgt,
                       std::vector<PointPlanePair>* const pairs,
                       double dist_sq_thresh) const;

  void Visualize(const FeaturePoints& feature,
                 const std::vector<PointLinePair>& pl_pairs,
                 const std::vector<PointPlanePair>& pp_pairs) const;

  Pose3d pose_curr2world_;
  Pose3d pose_curr2last_;

  TPointCloudPtr surf_pts_pre_{nullptr};
  TPointCloudPtr corn_pts_pre_{nullptr};

  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_surf_pts_{nullptr};
  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_corn_pts_{nullptr};

  bool is_initialized_ = false;
  bool is_vis_ = false;
  YAML::Node config_;

  std::unique_ptr<OdometryVisualizer> visualizer_{nullptr};

  DISALLOW_COPY_AND_ASSIGN(Odometry)
};

}  // namespace oh_my_loam
