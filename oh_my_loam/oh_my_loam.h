#pragma once

#include <yaml-cpp/node/node.h>

#include "common/common.h"
#include "oh_my_loam/extractor/extractor.h"
#include "oh_my_loam/mapper/mapper.h"
#include "oh_my_loam/odometer/odometer.h"
#include "oh_my_loam/visualizer/ohmyloam_visualizer.h"

namespace oh_my_loam {

class OhMyLoam {
 public:
  OhMyLoam() = default;

  bool Init();

  void Run(double timestamp, const common::PointCloudConstPtr &cloud_in);

 private:
  void Reset();

  void Visualize(const common::Pose3d &pose_curr2map,
                 const TPointCloudPtr &cloud_corn,
                 const TPointCloudPtr &cloud_surf, double timestamp = 0.0);

  // remove outliers: nan or very close points
  void RemoveOutliers(const common::PointCloud &cloud_in,
                      common::PointCloud *const cloud_out) const;

  std::unique_ptr<Extractor> extractor_{nullptr};

  std::unique_ptr<Odometer> odometer_{nullptr};

  std::unique_ptr<Mapper> mapper_{nullptr};

  YAML::Node config_;
  bool is_vis_ = false;

  std::unique_ptr<OhmyloamVisualizer> visualizer_{nullptr};

  DISALLOW_COPY_AND_ASSIGN(OhMyLoam);
};

}  // namespace oh_my_loam