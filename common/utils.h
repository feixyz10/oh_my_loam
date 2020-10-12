#pragma once

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
inline double NormalizeAngle(double ang) {
  const double& two_pi = 2 * M_PI;
  return ang - two_pi * std::floor((ang + M_PI) / two_pi);
}

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

}  // namespace oh_my_loam