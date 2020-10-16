#pragma once

#include "common.h"

namespace oh_my_loam {

struct FeaturePoints {
  IPointCloudPtr feature_pts;  // all feature points
  IPointCloudPtr sharp_corner_pts;
  IPointCloudPtr less_sharp_corner_pts;
  IPointCloudPtr flat_surf_pts;
  IPointCloudPtr less_flat_surf_pts;

  FeaturePoints() {
    feature_pts.reset(new IPointCloud);
    sharp_corner_pts.reset(new IPointCloud);
    less_sharp_corner_pts.reset(new IPointCloud);
    flat_surf_pts.reset(new IPointCloud);
    less_flat_surf_pts.reset(new IPointCloud);
  }
};

}  // namespace oh_my_loam
