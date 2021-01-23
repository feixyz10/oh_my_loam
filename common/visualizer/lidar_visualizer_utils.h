#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <string>

#include "common/color/color.h"
#include "common/pcl/pcl_types.h"

namespace common {

using PCLVisualizer = pcl::visualization::PCLVisualizer;
#define PCLColorHandlerCustom pcl::visualization::PointCloudColorHandlerCustom
#define PCLColorHandlerGenericField \
  pcl::visualization::PointCloudColorHandlerGenericField

template <typename PT>
void AddPointCloud(const typename pcl::PointCloud<PT>::ConstPtr &cloud,
                   const Color &color, const std::string &id,
                   PCLVisualizer *const viewer, int point_size = 3) {
  PCLColorHandlerCustom<PT> color_handler(cloud, color.r, color.g, color.b);
  viewer->addPointCloud<PT>(cloud, color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

template <typename PT>
void AddPointCloud(const typename pcl::PointCloud<PT>::ConstPtr &cloud,
                   const std::string &field, const std::string &id,
                   PCLVisualizer *const viewer, int point_size = 3) {
  PCLColorHandlerGenericField<PT> color_handler(cloud, field);
  viewer->addPointCloud<PT>(cloud, color_handler, id);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

void AddLine(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2,
             const Color &color, const std::string &id,
             PCLVisualizer *const viewer);

void AddSphere(const pcl::PointXYZ &center, double radius, const Color &color,
               const std::string &id, PCLVisualizer *const viewer);

}  // namespace common