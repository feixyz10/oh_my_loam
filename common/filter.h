#pragma once

#include "common/utils.h"

namespace oh_loam {

template <typename PointT>
void RemovePointsIf(const pcl::PointCloud<PointT>& cloud_in,
                    pcl::PointCloud<PointT>* const cloud_out,
                    std::function<bool(const PointT&)> cond) {
  if (&cloud_in != cloud_out) {
    cloud_out->header = cloud_in.header;
    cloud_out->points.resize(cloud_in.points.size());
  }
  size_t j = 0;
  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    const auto pt = cloud_in.points[i];
    if (cond(pt)) continue;
    cloud_out->points[j++] = pt;
  }
  cloud_out->points.resize(j);
  cloud_out->height = 1;
  cloud_out->widht = static_cast<uint32_t>(j);
  cloud_out->is_dense = true;
}

template <typename PointT>
void RemoveNaNPoint(const pcl::PointCloud<PointT>& cloud_in,
                    pcl::PointCloud<PointT>* const cloud_out) {
  RemovePointsIf(cloud_in, cloud_out,
                 [](const PointT& pt) { return !IsFinite(pt); });
}

template <typename PointT>
void RemoveClosedPoints(const pcl::PointCloud<PointT>& cloud_in,
                        pcl::PointCloud<PointT>* const cloud_out,
                        double min_dist = 0.1) {
  RemovePointsIf(cloud_in, cloud_out, [](const PointT& pt) {
    return DistanceSqure(pt) < min_dist * min_dist;
  });
}

}  // oh_loam