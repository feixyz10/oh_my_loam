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

  void Add(const FeaturePoints& rhs) {
    *sharp_corner_pts += *rhs.sharp_corner_pts;
    *less_sharp_corner_pts += *rhs.less_sharp_corner_pts;
    *flat_surf_pts += *rhs.flat_surf_pts;
    *less_flat_surf_pts += *rhs.less_flat_surf_pts;
  }
};

}  // namespace oh_my_loam
