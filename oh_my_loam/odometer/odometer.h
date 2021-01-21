#pragma once

#include "common/common.h"
#include "common/geometry/pose3d.h"
#include "oh_my_loam/base/feature.h"
#include "oh_my_loam/base/helper.h"
#include "oh_my_loam/visualizer/odometer_visualizer.h"

namespace oh_my_loam {

// frame to frame lidar Odometer
class Odometer {
 public:
  Odometer() = default;

  bool Init();

  void Process(double timestamp, const std::vector<Feature>& features,
               common::Pose3d* const pose);

 protected:
  void UpdatePre(const std::vector<Feature>& features);

  void MatchCorn(const common::TPointCloud& src,
                 std::vector<PointLinePair>* const pairs) const;

  void MatchSurf(const common::TPointCloud& src,
                 std::vector<PointPlanePair>* const pairs) const;

  void Visualize(const std::vector<Feature>& features,
                 const std::vector<PointLinePair>& pl_pairs,
                 const std::vector<PointPlanePair>& pp_pairs,
                 double timestamp = 0.0) const;

  void NearestKSearch(
      const std::vector<pcl::KdTreeFLANN<common::TPoint>>& kdtrees,
      const TPoint& query_pt, size_t k,
      std::vector<std::vector<int>>* const indices,
      std::vector<std::vector<float>>* const dists) const;

  common::Pose3d pose_curr2world_;
  common::Pose3d pose_curr2last_;

  std::vector<common::TPointCloudPtr> clouds_corn_pre_;
  std::vector<common::TPointCloudPtr> clouds_surf_pre_;

  std::vector<pcl::KdTreeFLANN<common::TPoint>> kdtrees_surf_;
  std::vector<pcl::KdTreeFLANN<common::TPoint>> kdtrees_corn_;

  YAML::Node config_;

  bool is_initialized_ = false;

  bool is_vis_ = false;

  bool verbose_ = false;

  std::unique_ptr<OdometerVisualizer> visualizer_{nullptr};

  DISALLOW_COPY_AND_ASSIGN(Odometer)
};

}  // namespace oh_my_loam
