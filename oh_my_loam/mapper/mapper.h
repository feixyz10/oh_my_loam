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

  void Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
               const TPointCloudConstPtr &cloud_surf,
               common::Pose3d *const pose_out);

  TPointCloudConstPtr cloud_corn_map() const {
    return cloud_corn_map_;
  }

  TPointCloudConstPtr cloud_surf_map() const {
    return cloud_corn_map_;
  }

  void Reset();

 private:
  void Visualize();

  TPointCloudPtr cloud_corn_map_;
  TPointCloudPtr cloud_surf_map_;

  YAML::Node config_;

  bool is_initialized_ = false;

  bool is_vis_ = false;

  bool verbose_ = false;

  DISALLOW_COPY_AND_ASSIGN(Mapper)
};

}  // namespace oh_my_loam