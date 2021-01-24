#pragma once

#include <ceres/ceres.h>

#include <eigen3/Eigen/Dense>
#include <type_traits>

#include "common/common.h"
#include "oh_my_loam/base/helper.h"

namespace oh_my_loam {

class PointLineCostFunction {
 public:
  PointLineCostFunction(const PointLinePair &pair, double time)
      : pair_(pair), time_(time){};

  template <typename T>
  bool operator()(const T *const r_quat, const T *const t_vec,
                  T *residual) const;

  static ceres::CostFunction *Create(const PointLinePair &pair, double time) {
    return new ceres::AutoDiffCostFunction<PointLineCostFunction, 3, 4, 3>(
        new PointLineCostFunction(pair, time));
  }

 private:
  PointLinePair pair_;
  double time_;

  DISALLOW_COPY_AND_ASSIGN(PointLineCostFunction)
};

class PointPlaneCostFunction {
 public:
  PointPlaneCostFunction(const PointPlanePair &pair, double time)
      : pair_(pair), time_(time){};

  template <typename T>
  bool operator()(const T *const r_quat, const T *const t_vec,
                  T *residual) const;

  static ceres::CostFunction *Create(const PointPlanePair &pair, double time) {
    return new ceres::AutoDiffCostFunction<PointPlaneCostFunction, 1, 4, 3>(
        new PointPlaneCostFunction(pair, time));
  }

 private:
  PointPlanePair pair_;
  double time_;
  DISALLOW_COPY_AND_ASSIGN(PointPlaneCostFunction)
};

template <typename T>
bool PointLineCostFunction::operator()(const T *const r_quat,
                                       const T *const t_vec,
                                       T *residual) const {
  const auto &pt = pair_.pt, &pt1 = pair_.line.pt1, &pt2 = pair_.line.pt2;
  Eigen::Matrix<T, 3, 1> p(T(pt.x), T(pt.y), T(pt.z));
  Eigen::Matrix<T, 3, 1> p1(T(pt1.x), T(pt1.y), T(pt1.z));
  Eigen::Matrix<T, 3, 1> p2(T(pt2.x), T(pt2.y), T(pt2.z));

  Eigen::Quaternion<T> r(r_quat[3], r_quat[0], r_quat[1], r_quat[2]);
  Eigen::Quaternion<T> r_interp =
      Eigen::Quaternion<T>::Identity().slerp(T(time_), r);
  Eigen::Matrix<T, 3, 1> t(T(time_) * t_vec[0], T(time_) * t_vec[1],
                           T(time_) * t_vec[2]);
  Eigen::Matrix<T, 3, 1> p0 = r_interp * p + t;

  // norm of cross product: triangle area x 2
  Eigen::Matrix<T, 3, 1> area = (p0 - p1).cross(p0 - p2) * 0.5;
  T base_length = (p2 - p1).norm();
  residual[0] = area[0] / base_length;
  residual[1] = area[1] / base_length;
  residual[2] = area[2] / base_length;
  if constexpr (!std::is_arithmetic<T>::value) {
    AERROR << p.transpose() << ", ";
  }
  return true;
}

template <typename T>
bool PointPlaneCostFunction::operator()(const T *const r_quat,
                                        const T *const t_vec,
                                        T *residual) const {
  const auto &pt = pair_.pt, &pt1 = pair_.plane.pt1, &pt2 = pair_.plane.pt2,
             &pt3 = pair_.plane.pt3;
  Eigen::Matrix<T, 3, 1> p(T(pt.x), T(pt.y), T(pt.z));
  Eigen::Matrix<T, 3, 1> p1(T(pt1.x), T(pt1.y), T(pt1.z));
  Eigen::Matrix<T, 3, 1> p2(T(pt2.x), T(pt2.y), T(pt2.z));
  Eigen::Matrix<T, 3, 1> p3(T(pt3.x), T(pt3.y), T(pt3.z));

  Eigen::Quaternion<T> r(r_quat[3], r_quat[0], r_quat[1], r_quat[2]);
  Eigen::Quaternion<T> r_interp =
      Eigen::Quaternion<T>::Identity().slerp(T(time_), r);
  Eigen::Matrix<T, 3, 1> t(T(time_) * t_vec[0], T(time_) * t_vec[1],
                           T(time_) * t_vec[2]);
  Eigen::Matrix<T, 3, 1> p0 = r_interp * p + t;

  Eigen::Matrix<T, 3, 1> normal = (p2 - p1).cross(p3 - p1).normalized();
  residual[0] = (p0 - p1).dot(normal);
  if constexpr (!std::is_arithmetic<T>::value) {
    AERROR << "ppres" << residual[0].a;
  }
  return true;
}

}  // namespace oh_my_loam