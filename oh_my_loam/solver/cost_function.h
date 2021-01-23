#pragma once

#include <ceres/ceres.h>

#include "common/common.h"
#include "oh_my_loam/base/helper.h"

namespace oh_my_loam {

class PointLineCostFunction {
 public:
  PointLineCostFunction(const PointLinePair &pair, double time)
      : pair_(pair), time_(time){};

  template <typename T>
  bool operator()(const T *const q, const T *const p, T *residual) const;

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
  bool operator()(const T *const q, const T *const p, T *residual) const;

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
bool PointLineCostFunction::operator()(const T *const q, const T *const p,
                                       T *residual) const {
  const auto pt = pair_.pt, pt1 = pair_.line.pt1, pt2 = pair_.line.pt2;
  Eigen::Matrix<T, 3, 1> pnt(T(pt.x), T(pt.y), T(pt.z));
  Eigen::Matrix<T, 3, 1> pnt1(T(pt1.x), T(pt1.y), T(pt1.z));
  Eigen::Matrix<T, 3, 1> pnt2(T(pt2.x), T(pt2.y), T(pt2.z));

  Eigen::Quaternion<T> r(q[3], q[0], q[1], q[2]);
  Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
  r = q_identity.slerp(T(time_), r);
  Eigen::Matrix<T, 3, 1> t(T(time_) * p[0], T(time_) * p[1], T(time_) * p[2]);
  Eigen::Matrix<T, 3, 1> pnt_n = r * pnt + t;

  // norm of cross product: triangle area x 2
  Eigen::Matrix<T, 3, 1> area = (pnt_n - pnt1).cross(pnt_n - pnt2) * 0.5;
  T base_length = (pnt2 - pnt1).norm();
  residual[0] = area[0] / base_length;
  residual[1] = area[1] / base_length;
  residual[2] = area[2] / base_length;
  return true;
}

template <typename T>
bool PointPlaneCostFunction::operator()(const T *const q, const T *const p,
                                        T *residual) const {
  const auto &pt = pair_.pt, &pt1 = pair_.plane.pt1, &pt2 = pair_.plane.pt2,
             &pt3 = pair_.plane.pt3;
  Eigen::Matrix<T, 3, 1> pnt(T(pt.x), T(pt.y), T(pt.z));
  Eigen::Matrix<T, 3, 1> pnt1(T(pt1.x), T(pt1.y), T(pt1.z));
  Eigen::Matrix<T, 3, 1> pnt2(T(pt2.x), T(pt2.y), T(pt2.z));
  Eigen::Matrix<T, 3, 1> pnt3(T(pt3.x), T(pt3.y), T(pt3.z));

  Eigen::Quaternion<T> r(q[3], q[0], q[1], q[2]);
  Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
  r = q_identity.slerp(T(time_), r);
  Eigen::Matrix<T, 3, 1> t(T(time_) * p[0], T(time_) * p[1], T(time_) * p[2]);
  Eigen::Matrix<T, 3, 1> pnt_n = r * pnt + t;

  Eigen::Matrix<T, 3, 1> normal = (pnt2 - pnt1).cross(pnt3 - pnt1).normalized();
  residual[0] = (pnt_n - pnt1).dot(normal);
  return true;
}

}  // namespace oh_my_loam