#pragma once

#include <vector>

#include "common/common.h"
#include "common/geometry/pose3d.h"
#include "oh_my_loam/base/feature.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

class Mapper {
 public:
  Mapper() = default;

  bool Init();

  void Process(double timestamp, const std::vector<Feature> &features,
               common::Pose3d *const pose_out);

  common::PointCloudConstPtr map() const {
    return cloud_map_;
  }

  void Reset();

 private:
  void Visualize();

  common::PointCloudPtr cloud_map_;

  common::Pose3d pose_curr2world_;

  YAML::Node config_;

  bool is_initialized = false;

  bool is_vis_ = false;

  bool verbose_ = false;

  DISALLOW_COPY_AND_ASSIGN(Mapper)
};

}  // namespace oh_my_loam