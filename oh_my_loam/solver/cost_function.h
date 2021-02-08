#pragma once

#include <ceres/ceres.h>

#include "common/common.h"
#include "oh_my_loam/base/utils.h"

namespace oh_my_loam {

struct PointLinePair {
  TPoint pt;
  struct Line {
    TPoint pt1, pt2;
    Line() = default;
    Line(const TPoint &pt1, const TPoint &pt2) : pt1(pt1), pt2(pt2) {}
  };
  Line line;
  PointLinePair(const TPoint &pt, const Line &line) : pt(pt), line(line) {}
  PointLinePair(const TPoint &pt, const TPoint &pt1, const TPoint &pt2)
      : pt(pt), line(pt1, pt2) {}
};

struct PointPlanePair {
  TPoint pt;
  struct Plane {
    TPoint pt1, pt2, pt3;
    Plane() = default;
    Plane(const TPoint &pt1, const TPoint &pt2, const TPoint &pt3)
        : pt1(pt1), pt2(pt2), pt3(pt3) {}
  };
  Plane plane;
  PointPlanePair(const TPoint &pt, const Plane &plane) : pt(pt), plane(plane) {}
  PointPlanePair(const TPoint &pt, const TPoint &pt1, const TPoint &pt2,
                 const TPoint &pt3)
      : pt(pt), plane(pt1, pt2, pt3) {}
};

struct PointLineCoeffPair {
  TPoint pt;
  Eigen::Matrix<double, 6, 1> line_coeff;
  PointLineCoeffPair(const TPoint &pt,
                     const Eigen::Matrix<double, 6, 1> &line_coeff)
      : pt(pt), line_coeff(line_coeff) {}
};

struct PointPlaneCoeffPair {
  TPoint pt;
  Eigen::Vector4d plane_coeff;
  PointPlaneCoeffPair(const TPoint &pt, const Eigen::Vector4d &plane_coeff)
      : pt(pt), plane_coeff(plane_coeff) {}
};

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

class PointLineCoeffCostFunction {
 public:
  PointLineCoeffCostFunction(const PointLineCoeffPair &pair, double time)
      : pair_(pair), time_(time){};

  template <typename T>
  bool operator()(const T *const r_quat, const T *const t_vec,
                  T *residual) const;

  static ceres::CostFunction *Create(const PointLineCoeffPair &pair,
                                     double time) {
    return new ceres::AutoDiffCostFunction<PointLineCoeffCostFunction, 1, 4, 3>(
        new PointLineCoeffCostFunction(pair, time));
  }

 private:
  PointLineCoeffPair pair_;
  double time_;
  DISALLOW_COPY_AND_ASSIGN(PointLineCoeffCostFunction)
};

class PointPlaneCoeffCostFunction {
 public:
  PointPlaneCoeffCostFunction(const PointPlaneCoeffPair &pair, double time)
      : pair_(pair), time_(time){};

  template <typename T>
  bool operator()(const T *const r_quat, const T *const t_vec,
                  T *residual) const;

  static ceres::CostFunction *Create(const PointPlaneCoeffPair &pair,
                                     double time) {
    return new ceres::AutoDiffCostFunction<PointPlaneCoeffCostFunction, 1, 4,
                                           3>(
        new PointPlaneCoeffCostFunction(pair, time));
  }

 private:
  PointPlaneCoeffPair pair_;
  double time_;
  DISALLOW_COPY_AND_ASSIGN(PointPlaneCoeffCostFunction)
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
  Eigen::Matrix<T, 3, 1> t(t_vec[0], t_vec[1], t_vec[2]);
  if (time_ <= 1.0 - 1.0e-6) {
    r = Eigen::Quaternion<T>::Identity().slerp(T(time_), r);
    t = T(time_) * t;
  }
  Eigen::Matrix<T, 3, 1> p0 = r * p + t;

  // norm of cross product: triangle area x 2
  Eigen::Matrix<T, 3, 1> area = (p0 - p1).cross(p0 - p2) * 0.5;
  T base_length = (p2 - p1).norm();
  residual[0] = area[0] / base_length;
  residual[1] = area[1] / base_length;
  residual[2] = area[2] / base_length;
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
  Eigen::Matrix<T, 3, 1> t(t_vec[0], t_vec[1], t_vec[2]);
  if (time_ <= 1.0 - 1.0e-6) {
    r = Eigen::Quaternion<T>::Identity().slerp(T(time_), r);
    t = T(time_) * t;
  }
  Eigen::Matrix<T, 3, 1> p0 = r * p + t;

  Eigen::Matrix<T, 3, 1> normal = (p2 - p1).cross(p3 - p1).normalized();
  residual[0] = (p0 - p1).dot(normal);
  return true;
}

template <typename T>
bool PointLineCoeffCostFunction::operator()(const T *const r_quat,
                                            const T *const t_vec,
                                            T *residual) const {
  Eigen::Matrix<T, 3, 1> p(T(pair_.pt.x), T(pair_.pt.y), T(pair_.pt.z));
  Eigen::Matrix<T, 6, 1> coeff = pair_.line_coeff.template cast<T>();
  Eigen::Matrix<T, 3, 1> p1 = coeff.topRows(3);
  Eigen::Matrix<T, 3, 1> dir = coeff.bottomRows(3);

  Eigen::Quaternion<T> r(r_quat[3], r_quat[0], r_quat[1], r_quat[2]);
  Eigen::Matrix<T, 3, 1> t(t_vec[0], t_vec[1], t_vec[2]);
  if (time_ <= 1.0 - 1.0e-6) {
    r = Eigen::Quaternion<T>::Identity().slerp(T(time_), r);
    t = T(time_) * t;
  }
  Eigen::Matrix<T, 3, 1> p0 = r * p + t;

  Eigen::Matrix<T, 3, 1> cross = (p0 - p1).cross(dir);
  residual[0] = cross[0];
  residual[1] = cross[1];
  residual[2] = cross[2];
  return true;
}

template <typename T>
bool PointPlaneCoeffCostFunction::operator()(const T *const r_quat,
                                             const T *const t_vec,
                                             T *residual) const {
  Eigen::Matrix<T, 3, 1> p(T(pair_.pt.x), T(pair_.pt.y), T(pair_.pt.z));
  Eigen::Matrix<T, 4, 1> coeff = pair_.plane_coeff.template cast<T>();

  Eigen::Quaternion<T> r(r_quat[3], r_quat[0], r_quat[1], r_quat[2]);
  Eigen::Matrix<T, 3, 1> t(t_vec[0], t_vec[1], t_vec[2]);
  if (time_ <= 1.0 - 1.0e-6) {
    r = Eigen::Quaternion<T>::Identity().slerp(T(time_), r);
    t = T(time_) * t;
  }
  Eigen::Matrix<T, 3, 1> p0 = r * p + t;

  residual[0] = coeff.topRows(3).transpose() * p0;
  residual[0] += coeff(3);
  return true;
}

}  // namespace oh_my_loam