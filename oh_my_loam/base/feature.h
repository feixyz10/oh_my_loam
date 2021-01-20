#pragma once

#include "common/pcl/pcl_types.h"

namespace oh_my_loam {

struct Feature {
  common::TPointCloudPtr cloud_corner;
  common::TPointCloudPtr cloud_sharp_corner;
  common::TPointCloudPtr cloud_surf;
  common::TPointCloudPtr cloud_flat_surf;

  Feature() {
    cloud_corner.reset(new common::TPointCloud);
    cloud_sharp_corner.reset(new common::TPointCloud);
    cloud_surf.reset(new common::TPointCloud);
    cloud_flat_surf.reset(new common::TPointCloud);
  }
};

}  // namespace oh_my_loam
