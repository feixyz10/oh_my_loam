#pragma once

#include "common.h"
#include "extractor/base_extractor.h"
#include "odometry/odometry.h"

namespace oh_my_loam {

class OhMyLoam {
 public:
  OhMyLoam() = default;

  bool Init();

  void Run(const PointCloud& cloud_in, double timestamp);

 private:
  std::unique_ptr<Extractor> extractor_{nullptr};
  std::unique_ptr<Odometry> odometry_{nullptr};

  // remove outliers: nan and very close points
  void RemoveOutliers(const PointCloud& cloud_in,
                      PointCloud* const cloud_out) const;
  std::vector<Pose3D> poses_;

  DISALLOW_COPY_AND_ASSIGN(OhMyLoam)
};

}  // namespace oh_my_loam