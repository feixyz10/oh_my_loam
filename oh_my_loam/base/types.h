#pragma once

#include "common/pcl/pcl_types.h"

namespace oh_my_loam {

enum class PType {
  FLAT_SURF = -2,
  SURF = -1,
  NORNAL = 0,
  CORNER = 1,
  SHARP_CORNER = 2
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

  PointXYZT(const common::Point &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = 0.0f;
  }

  PointXYZT(const PointXYZT &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointXYZTCT {
  PCL_ADD_POINT4D;
  union EIGEN_ALIGN16 {
    struct {
      float time;
      float curvature;
      PType type;
    };
    float data_c[4];
  };

  PointXYZTCT() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    time = curvature = 0.0f;
    type = PType::NORNAL;
  }

  PointXYZTCT(float x, float y, float z, float time = 0.0f,
              float curvature = 0.0f, PType type = PType::NORNAL)
      : x(x), y(y), z(z), time(time), curvature(curvature), type(type) {
    data[3] = 1.0f;
  }

  PointXYZTCT(const common::Point &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = 0.0f;
    curvature = 0.0f;
    type = PType::NORNAL;
  }

  PointXYZTCT(const TPoint &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
    curvature = 0.0f;
    type = PType::NORNAL;
  }

  PointXYZTCT(const PointXYZTCT &p) {
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
  oh_my_loam::PointXYZT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, time, time)
  (float, intensity, intensity)
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