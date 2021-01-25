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

  bool IsMapping(double timestamp) const;

  std::unique_ptr<Extractor> extractor_{nullptr};

  std::unique_ptr<Odometer> odometer_{nullptr};

  std::unique_ptr<Mapper> mapper_{nullptr};

  // remove outliers: nan or very close points
  void RemoveOutliers(const common::PointCloud &cloud_in,
                      common::PointCloud *const cloud_out) const;

  std::vector<common::Pose3d> poses_;

  YAML::Node config_;

  double timestamp_last_ = 0.0, timestamp_last_mapping_ = 0.0;

  DISALLOW_COPY_AND_ASSIGN(OhMyLoam)
};

}  // namespace oh_my_loam