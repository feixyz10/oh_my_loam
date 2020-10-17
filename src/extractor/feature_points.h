#pragma once

#include "common.h"

namespace oh_my_loam {

struct FeaturePoints {
  PointCloudPtr sharp_corner_pts;
  PointCloudPtr less_sharp_corner_pts;
  PointCloudPtr flat_surf_pts;
  PointCloudPtr less_flat_surf_pts;

  FeaturePoints() {
    sharp_corner_pts.reset(new PointCloud);
    less_sharp_corner_pts.reset(new PointCloud);
    flat_surf_pts.reset(new PointCloud);
    less_flat_surf_pts.reset(new PointCloud);
  }

  void Add(const FeaturePoints& rhs) {
    *sharp_corner_pts += *rhs.sharp_corner_pts;
    *less_sharp_corner_pts += *rhs.less_sharp_corner_pts;
    *flat_surf_pts += *rhs.flat_surf_pts;
    *less_flat_surf_pts += *rhs.less_flat_surf_pts;
  }
};

}  // namespace oh_my_loam
