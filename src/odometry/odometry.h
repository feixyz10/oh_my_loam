#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>

#include "common.h"
#include "helper/helper.h"

namespace oh_my_loam {

// frame to frame lidar odometry
class Odometry {
 public:
  Odometry() = default;

  bool Init(const YAML::Node& config);

  void Process(const FeaturePoints& feature);

 protected:
  void UpdatePre(const FeaturePoints& feature);
  void AssociateCornPoints(const TPointCloud& src, const TPointCloud& tgt,
                           std::vector<PointLinePair>* const pairs,
                           double dist_thresh) const;

  void AssociateSurfPoints(const TPointCloud& src, const TPointCloud& tgt,
                           std::vector<PointPlanePair>* const pairs,
                           double dist_thresh) const;

  TPointCloudPtr surf_pts_pre_;
  TPointCloudPtr corn_pts_pre_;

  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_surf_pts;
  pcl::KdTreeFLANN<TPoint>::Ptr kdtree_corn_pts;

  bool is_initialized = false;

  YAML::Node config_;
};

}  // namespace oh_my_loam
