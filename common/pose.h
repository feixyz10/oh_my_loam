#pragma once

#include <eigen3/Eigen/Dense>

namespace oh_my_loam {

class Pose3D {
 public:
  Pose3D() {
    q_.setIdentity();
    p_.setZero();
  };

  Pose3D(const Eigen::Quaterniond& q, const Eigen::Vector3d& p)
      : q_(q), p_(p) {}

  Pose3D(const Eigen::Matrix3d& r_mat, const Eigen::Vector3d& p) {
    q_ = Eigen::Quaterniond(r_mat);
    p_ = p;
  }

  Pose3D Inv() const {
    auto q_inv = q_.inverse();
    auto p_inv = q_inv * p_;
    return {q_inv, p_inv};
  }

  Eigen::Vector3d Transform(const Eigen::Vector3d& vec) const {
    return q_ * vec + p_;
  }

  template <typename PointT>
  PointT Transform(const PointT& pt) const {
    PointT pt_n = pt;
    Eigen::Vector3d vec = Transform(Eigen::Vector3d(pt.x, pt.y, pt.z));
    pt_n.x = vec.x;
    pt_n.y = vec.y;
    pt_n.z = vec.z;
    return pt_n;
  }

  Eigen::Vector3d Translate(const Eigen::Vector3d& vec) const {
    return vec + p_;
  }

  Eigen::Vector3d Rotate(const Eigen::Vector3d& vec) const { return q_ * vec; }

  // Spherical linear interpolation to `pose_to`, `t` belongs [0, 1]
  Pose3D InterPolate(const Pose3D& pose_to, double t) const {
    auto q_interp = q_.slerp(t, pose_to.q_);
    auto p_interp = (pose_to.p_ - p_) * t;
    return {q_interp, p_interp};
  }

 protected:
  Eigen::Quaterniond q_;  // orientation
  Eigen::Vector3d p_;     // position
};

Pose3D Interpolate(const Pose3D& pose_from, const Pose3D& pose_to, double t);

}  // namespace oh_my_loam
