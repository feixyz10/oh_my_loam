#pragma once

#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

struct Feature {
  TPointCloudPtr cloud_corner;
  TPointCloudPtr cloud_sharp_corner;
  TPointCloudPtr cloud_surf;
  TPointCloudPtr cloud_flat_surf;

  Feature() {
    cloud_corner.reset(new TPointCloud);
    cloud_sharp_corner.reset(new TPointCloud);
    cloud_surf.reset(new TPointCloud);
    cloud_flat_surf.reset(new TPointCloud);
  }
};

}  // namespace oh_my_loam
