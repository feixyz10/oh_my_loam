#pragma once

#include "common/log/log.h"
#include "pcl_types.h"

namespace common {

// Distance squred of a point to origin
template <typename PT>
inline double DistanceSqure(const PT &pt) {
  return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
}

// Distance squred of two points
template <typename PT>
inline double DistanceSqure(const PT &pt1, const PT &pt2) {
  return (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) +
         (pt1.z - pt2.z) * (pt1.z - pt2.z);
}

// Distance of a point to origin
template <typename PT>
inline double Distance(const PT &pt) {
  return std::sqrt(DistanceSqure(pt));
}

// Distance squred of two points
template <typename PT>
inline double Distance(const PT &pt1, const PT &pt2) {
  return std::sqrt(DistanceSqure(pt1, pt2));
}

// Check whether is a finite point: neither infinite nor nan
template <typename PT>
inline double IsFinite(const PT &pt) {
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}

// Remove point if the condition evaluated to true on it
template <typename PT>
void RemovePoints(const pcl::PointCloud<PT> &cloud_in,
                  pcl::PointCloud<PT> *const cloud_out,
                  std::function<bool(const PT &)> check,
                  std::vector<int> *const removed_indices = nullptr) {
  if (&cloud_in != cloud_out) {
    cloud_out->header = cloud_in.header;
    cloud_out->resize(cloud_in.size());
  }
  size_t j = 0;
  for (size_t i = 0; i < cloud_in.size(); ++i) {
    const auto pt = cloud_in.points[i];
    if (check(pt)) {
      if (removed_indices) removed_indices->push_back(i);
      continue;
    }
    cloud_out->points[j++] = pt;
  }

  cloud_out->points.resize(j);
  cloud_out->height = 1;
  cloud_out->width = static_cast<uint32_t>(j);
  cloud_out->is_dense = true;
}

template <typename PT>
void VoxelDownSample(const typename pcl::PointCloud<PT>::ConstPtr &cloud_in,
                     pcl::PointCloud<PT> *const cloud_out, double voxel_size) {
  pcl::VoxelGrid<PT> filter;
  filter.setInputCloud(cloud_in);
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.filter(*cloud_out);
}

}  // namespace common