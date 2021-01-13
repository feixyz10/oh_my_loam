#pragma once

#include "common/common.h"
#include "common/geometry/pose.h"
#include "oh_my_loam/base/feature.h"
#include "oh_my_loam/base/helper.h"
#include "oh_my_loam/visualizer/odometer_visualizer.h"

namespace oh_my_loam {

// frame to frame lidar Odometer
class Odometer {
 public:
  Odometer() = default;

  bool Init();

  void Process(double timestamp, const Feature& feature,
               common::Pose3D* const pose);

 protected:
  void UpdatePre(const Feature& feature);

  void MatchCornFeature(const common::TPointCloud& src,
                        const common::TPointCloud& tgt,
                        std::vector<PointLinePair>* const pairs) const;

  void MatchSurfFeature(const common::TPointCloud& src,
                        const common::TPointCloud& tgt,
                        std::vector<PointPlanePair>* const pairs) const;

  void Visualize(const Feature& feature,
                 const std::vector<PointLinePair>& pl_pairs,
                 const std::vector<PointPlanePair>& pp_pairs,
                 double timestamp = 0.0) const;

  common::Pose3D pose_curr2world_;
  common::Pose3D pose_curr2last_;

  common::TPointCloudPtr corn_pre_{nullptr};
  common::TPointCloudPtr surf_pre_{nullptr};

  pcl::KdTreeFLANN<common::TPoint>::Ptr kdtree_surf_{nullptr};
  pcl::KdTreeFLANN<common::TPoint>::Ptr kdtree_corn_{nullptr};

  YAML::Node config_;

  bool is_initialized_ = false;

  bool is_vis_ = false;

  bool verbose_ = false;

  std::unique_ptr<OdometerVisualizer> visualizer_{nullptr};

  DISALLOW_COPY_AND_ASSIGN(Odometer)
};

}  // namespace oh_my_loam
