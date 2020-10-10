#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

namespace oh_loam {

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

struct EIGEN_ALIGN16 PointXYZTCT {
  PCL_ADD_POINT4D;
  float time = 0.0f;
  float curvature = NAN;
  PointType type = PointType::NORNAL;

  PointXYZTCT() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
  }

  PointXYZTCT(float x, float y, float z, float time = 0.0f,
              float curvature = NAN, PointType type = PointType::NORNAL)
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using IPoint = PointXYZTCT;
using IPointCloud = pcl::PointCloud<IPoint>;
using IPointCloudPtr = IPointCloud::Ptr;
using IPointCloudConstPtr = IPointCloud::ConstPtr;

}  // oh_loam