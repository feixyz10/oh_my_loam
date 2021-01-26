#pragma once

#include <yaml-cpp/node/node.h>

#include "common/common.h"
#include "oh_my_loam/extractor/extractor.h"
#include "oh_my_loam/mapper/mapper.h"
#include "oh_my_loam/odometer/odometer.h"

namespace oh_my_loam {

class OhMyLoam {
 public:
  OhMyLoam() = default;

  bool Init();

  void Run(double timestamp, const common::PointCloudConstPtr &cloud_in);

 private:
  void Reset();

  void FusionOdometryMapping();

  void MappingProcess(double timestamp, const TPointCloudConstPtr &cloud_corn,
                      const TPointCloudConstPtr &cloud_surf);

  void Visualize(double timestamp = 0.0);

  std::unique_ptr<Extractor> extractor_{nullptr};

  std::unique_ptr<Odometer> odometer_{nullptr};

  std::unique_ptr<Mapper> mapper_{nullptr};

  // remove outliers: nan or very close points
  void RemoveOutliers(const common::PointCloud &cloud_in,
                      common::PointCloud *const cloud_out) const;

  struct TimePose {
    double timestamp;
    common::Pose3d pose;
  };

  std::vector<TimePose> poses_curr2world_;

  std::mutex mutex_;
  TimePose pose_mapping_;
  bool pose_mapping_updated_ = true;
  std::unique_ptr<std::thread> mapping_thread_{nullptr};

  YAML::Node config_;

  DISALLOW_COPY_AND_ASSIGN(OhMyLoam)
};

}  // namespace oh_my_loam