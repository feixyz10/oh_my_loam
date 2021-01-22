#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Following hpp file should be included if user-defined point type is added,
// see "How are the point types exposed?" section in
// https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace common {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

using Indices = std::vector<int>;

using Point2D = pcl::PointXY;
using PointCloud2D = pcl::PointCloud<Point2D>;
using PointCloud2DPtr = PointCloud2D::Ptr;
using PointCloud2DConstPtr = PointCloud2D::ConstPtr;

}  // namespace common