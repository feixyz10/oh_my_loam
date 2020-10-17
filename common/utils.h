#pragma once

#include "color.h"
#include "log.h"
#include "types.h"

namespace oh_my_loam {

template <typename PointT>
inline double DistanceSqure(const PointT& pt) {
  return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
}

template <typename PointT>
inline double Distance(const PointT& pt) {
  return std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

template <typename PointT>
inline double IsFinite(const PointT& pt) {
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}

// normalize an angle to [-pi, pi)
double NormalizeAngle(double ang);

// like Python built-in range, [begin, end)
const std::vector<int> Range(int begin, int end, int step = 1);
const std::vector<int> Range(int end);  // [0, end)

template <typename PointT>
void DrawPointCloud(const pcl::PointCloud<PointT>& cloud, const Color& color,
                    const std::string& id, PCLVisualizer* const viewer,
                    int pt_size = 3) {
  PCLColorHandlerCustom<PointT> color_handler(cloud.makeShared(), color.r,
                                              color.g, color.b);
  viewer->addPointCloud<PointT>(cloud.makeShared(), color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, id);
}

template <typename PointT>
void DrawPointCloud(const pcl::PointCloud<PointT>& cloud,
                    const std::string& field, const std::string& id,
                    PCLVisualizer* const viewer, int pt_size = 3) {
  PCLColorHandlerGenericField<PointT> color_handler(cloud.makeShared(), field);
  viewer->addPointCloud<PointT>(cloud.makeShared(), color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, id);
}

template <typename PointT>
void RemovePointsIf(const pcl::PointCloud<PointT>& cloud_in,
                    pcl::PointCloud<PointT>* const cloud_out,
                    std::function<bool(const PointT&)> cond) {
  if (&cloud_in != cloud_out) {
    cloud_out->header = cloud_in.header;
    cloud_out->points.resize(cloud_in.size());
  }
  size_t j = 0;
  for (size_t i = 0; i < cloud_in.size(); ++i) {
    const auto pt = cloud_in.points[i];
    if (cond(pt)) continue;
    cloud_out->points[j++] = pt;
  }

  cloud_out->points.resize(j);
  cloud_out->height = 1;
  cloud_out->width = static_cast<uint32_t>(j);
  cloud_out->is_dense = true;
}

template <typename PointT>
void RemoveNaNPoint(const pcl::PointCloud<PointT>& cloud_in,
                    pcl::PointCloud<PointT>* const cloud_out) {
  RemovePointsIf<PointT>(cloud_in, cloud_out,
                         [](const PointT& pt) { return !IsFinite(pt); });
}

template <typename PointT>
void RemoveClosedPoints(const pcl::PointCloud<PointT>& cloud_in,
                        pcl::PointCloud<PointT>* const cloud_out,
                        double min_dist = 0.1) {
  RemovePointsIf<PointT>(cloud_in, cloud_out, [&](const PointT& pt) {
    return DistanceSqure(pt) < min_dist * min_dist;
  });
}

template <typename PointT>
void VoxelDownSample(const pcl::PointCloud<PointT>& cloud_in,
                     pcl::PointCloud<PointT>* const cloud_out,
                     double voxel_size) {
  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud(cloud_in.makeShared());
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.filter(*cloud_out);
}

}  // namespace oh_my_loam