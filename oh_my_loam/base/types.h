#pragma once

#include "common/pcl/pcl_types.h"

namespace oh_my_loam {

enum class Type {
  FLAT = -2,
  LESS_FLAT = -1,
  NORNAL = 0,
  LESS_SHARP = 1,
  SHARP = 2,
};

struct PointXYZTCT;
using TCTPoint = PointXYZTCT;
using TCTPointCloud = pcl::PointCloud<TCTPoint>;
using TCTPointCloudPtr = TCTPointCloud::Ptr;
using TCTPointCloudConstPtr = TCTPointCloud::ConstPtr;

struct PointXYZTCT {
  PCL_ADD_POINT4D;
  union EIGEN_ALIGN16 {
    struct {
      float time;
      float curvature;
      Type type;
    };
    float data_c[4];
  };

  PointXYZTCT() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    time = curvature = 0.0f;
    type = Type::NORNAL;
  }

  PointXYZTCT(float x, float y, float z, float time = 0.0f,
              float curvature = 0.0f, Type type = Type::NORNAL)
      : x(x), y(y), z(z), time(time), curvature(curvature), type(type) {
    data[3] = 1.0f;
  }

  PointXYZTCT(const common::Point& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = 0.0f;
    curvature = 0.0f;
    type = Type::NORNAL;
  }

  PointXYZTCT(const common::TPoint& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
    curvature = 0.0f;
    type = Type::NORNAL;
  }

  PointXYZTCT(const PointXYZTCT& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
    curvature = p.curvature;
    type = p.type;
  }

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
  (float, curvature, curvature)
)
// clang-format on