#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <cmath>
// This hpp file should be included if user-defined point type is added, see
// "How are the point types exposed?" section in
// https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html
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

struct Color {
  uint8_t r, g, b;
  Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
};
#define BLACK Color(0, 0, 0)
#define WHITE Color(255, 255, 255)
#define RED Color(255, 0, 0)
#define GREEN Color(0, 255, 0)
#define BLUE Color(0, 0, 255)
#define GRAY Color(50, 50, 50)
#define CYAN Color(0, 255, 255)
#define PURPLE Color(160, 32, 240)
#define ORANGE Color(255, 97, 0)

struct PointXYZTCT;

using IPoint = PointXYZTCT;
using IPointCloud = pcl::PointCloud<IPoint>;
using IPointCloudPtr = IPointCloud::Ptr;
using IPointCloudConstPtr = IPointCloud::ConstPtr;

using PCLVisualizer = pcl::visualization::PCLVisualizer;
#define PCLColorHandlerCustom pcl::visualization::PointCloudColorHandlerCustom
#define PCLColorHandlerGenericField \
  pcl::visualization::PointCloudColorHandlerGenericField

struct PointXYZTCT {
  PCL_ADD_POINT4D;
  float time = 0.0f;
  float curvature = std::nanf("");
  int8_t type = 0;  // -2, -1, 0, 1, or 2

  PointXYZTCT() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
  }

  PointXYZTCT(float x, float y, float z, float time = 0.0f,
              float curvature = NAN, int8_t type = 0)
      : x(x), y(y), z(z), time(time), curvature(curvature), type(type) {
    data[3] = 1.0f;
  }

  PointXYZTCT(const Point& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
  }

  PointXYZTCT(const PointXYZTCT& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    time = p.time;
    curvature = p.curvature;
    type = p.type;
    data[3] = 1.0f;
  }

  PointType Type() const { return static_cast<PointType>(type); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace oh_my_loam

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  oh_my_loam::PointXYZTCT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, time, time)
  (float, curvature, curvatur)
  (int8_t, type, type)
)
// clang-format on