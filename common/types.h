#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <cmath>
// Thses hpp file should be included if user-defined point type is added, see
// "How are the point types exposed?" section in
// https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

namespace oh_my_loam {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

enum class PointType {
  FLAT = -2,
  LESS_FLAT = -1,
  NORNAL = 0,
  LESS_SHARP = 1,
  SHARP = 2,
};

struct PointXYZT;
using TPoint = PointXYZT;
using TPointCloud = pcl::PointCloud<TPoint>;
using TPointCloudPtr = TPointCloud::Ptr;
using TPointCloudConstPtr = TPointCloud::ConstPtr;

struct PointXYZTCT;
using TCTPoint = PointXYZTCT;
using TCTPointCloud = pcl::PointCloud<TCTPoint>;
using TCTPointCloudPtr = TCTPointCloud::Ptr;
using TCTPointCloudConstPtr = TCTPointCloud::ConstPtr;

struct PointXYZT {
  PCL_ADD_POINT4D;
  union EIGEN_ALIGN16 {
    float time;
    // make sure VoxelGrid can work with this custom point type:
    // https://github.com/PointCloudLibrary/pcl/issues/2331
    float intensity;
  };

  PointXYZT() {
    x = y = z = 0.0f;
    time = 0.0f;
  }

  PointXYZT(float x, float y, float z, float time = 0.0f)
      : x(x), y(y), z(z), time(time) {}

  PointXYZT(const Point& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    time = 0.0f;
  }

  PointXYZT(const PointXYZT& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    time = p.time;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZTCT {
  PCL_ADD_POINT4D;
  union EIGEN_ALIGN16 {
    float data_c[4];
    struct {
      float time;
      float curvature;
      PointType type;
    };
  };

  PointXYZTCT() {
    x = y = z = 0.0f;
    time = 0.0f;
    curvature = std::nanf("");
    type = PointType::NORNAL;
  }

  PointXYZTCT(float x, float y, float z, float time = 0.0f,
              float curvature = std::nanf(""),
              PointType type = PointType::NORNAL)
      : x(x), y(y), z(z), time(time), curvature(curvature), type(type) {}

  PointXYZTCT(const Point& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    time = 0.0f;
    curvature = std::nanf("");
    type = PointType::NORNAL;
  }

  PointXYZTCT(const PointXYZTCT& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    time = p.time;
    curvature = p.curvature;
    type = p.type;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace oh_my_loam

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  oh_my_loam::PointXYZT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, time, time)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
  oh_my_loam::PointXYZTCT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, time, time)
  (float, curvature, curvature)
)
// clang-format on