#pragma once

#include "common/pcl/pcl_types.h"

namespace oh_my_loam {

struct Feature {
  common::TPointCloudPtr cloud_sharp_corner;
  common::TPointCloudPtr cloud_less_sharp_corner;
  common::TPointCloudPtr cloud_flat_surf;
  common::TPointCloudPtr cloud_less_flat_surf;

  Feature() {
    cloud_sharp_corner.reset(new common::TPointCloud);
    cloud_less_sharp_corner.reset(new common::TPointCloud);
    cloud_flat_surf.reset(new common::TPointCloud);
    cloud_less_flat_surf.reset(new common::TPointCloud);
  }
};

}  // namespace oh_my_loam
