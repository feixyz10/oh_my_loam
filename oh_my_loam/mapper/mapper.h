#pragma once

#include "common/common.h"
#include "common/geometry/pose.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

class Mapper {
 public:
  Mapper() = default;

  bool Init();

  void Process();

 private:
  void Visualize();

  TPointCloudPtr map_pts_;
  common::Pose3d pose_curr2world_;

  bool is_initialized = false;

  bool is_vis_ = false;

  bool verbose_ = false;

  YAML::Node config_;

  DISALLOW_COPY_AND_ASSIGN(Mapper)
};

}  // namespace oh_my_loam