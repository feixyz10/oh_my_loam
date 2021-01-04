#pragma once

#include "common.h"

namespace oh_my_loam {

class Mapper {
 public:
  Mapper() = default;

  bool Init(const YAML::Node& config);

  void Process();

 private:
  void Visualize();

  TPointCloudPtr map_pts_;
  Pose3d pose_curr2world_;

  bool is_initialized = false;
  bool is_vis_ = false;

  YAML::Node config_;

  DISALLOW_COPY_AND_ASSIGN(Mapper)
};

}  // namespace oh_my_loam