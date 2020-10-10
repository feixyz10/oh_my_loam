#pragma once

#include "types.h"

namespace oh_loam {

template <typename PointT>
inline double Distance(const PointT& pt) {
  return std::hypot(pt.x, pt.y, pt.z);
}

template <typename PointT>
inline double DistanceSqure(const PointT& pt) {
  return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
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

std::pair<double, double> GetYawRange(const PointCloud& cloud) {
  const auto& pts = cloud.points;
  int pt_num = pts.size();
  double yaw_start = -atan2(pts[0].y, pts[0].x);
  double yaw_end = -atan2(pts[pt_num - 1].y, pts[pt_num - 1].x) + 2 * M_PI;
  double yaw_diff = NormalizeAngle(yaw_end - yaw_start);
  return {yaw_start, yaw_start + yaw_diff + 2 * M_PI};
}

}  // oh_loam