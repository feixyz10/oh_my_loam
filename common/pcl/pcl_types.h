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

struct PointXYZT;
using TPoint = PointXYZT;
using TPointCloud = pcl::PointCloud<TPoint>;
using TPointCloudPtr = TPointCloud::Ptr;
using TPointCloudConstPtr = TPointCloud::ConstPtr;

struct EIGEN_ALIGN16 PointXYZT {
  PCL_ADD_POINT4D;
  union {
    float time;
    float intensity;
    float data_c[4];
  };

  PointXYZT() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    time = 0.0f;
  }

  PointXYZT(float x, float y, float z, float time = 0.0f)
      : x(x), y(y), z(z), time(time) {
    data[3] = 1.0f;
  }

  PointXYZT(const Point& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = 0.0f;
  }

  PointXYZT(const PointXYZT& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  ::common::PointXYZT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, time, time)
  // (float, intensity, intensity)
)
// clang-format on