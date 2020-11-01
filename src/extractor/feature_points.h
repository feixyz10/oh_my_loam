#pragma once

#include "types.h"

namespace oh_my_loam {

struct FeaturePoints {
  TPointCloudPtr sharp_corner_pts;
  TPointCloudPtr less_sharp_corner_pts;
  TPointCloudPtr flat_surf_pts;
  TPointCloudPtr less_flat_surf_pts;

  FeaturePoints() {
    sharp_corner_pts.reset(new TPointCloud);
    less_sharp_corner_pts.reset(new TPointCloud);
    flat_surf_pts.reset(new TPointCloud);
    less_flat_surf_pts.reset(new TPointCloud);
  }
};

}  // namespace oh_my_loam
