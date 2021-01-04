#pragma once

#include <eigen3/Eigen/Dense>

namespace common {

class Pose3d {
 public:
  Pose3d() {
    q_.setIdentity();
    p_.setZero();
  };

  Pose3d(const Eigen::Quaterniond& q, const Eigen::Vector3d& p)
      : q_(q), p_(p) {}

  Pose3d(const Eigen::Matrix3d& r_mat, const Eigen::Vector3d& p)
      : q_(r_mat), p_(p) {}

  Pose3d(const double* const q, const double* const p) : q_(q), p_(p) {}

  Pose3d Inv() const {
    Eigen::Quaterniond q_inv = q_.inverse();
    Eigen::Vector3d p_inv = q_inv * p_;
    return {q_inv, -p_inv};
  }

  Eigen::Vector3d Transform(const Eigen::Vector3d& vec) const {
    return q_ * vec + p_;
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d& vec) const {
    return Transform(vec);
  }

  Eigen::Vector3d Translate(const Eigen::Vector3d& vec) const {
    return vec + p_;
  }

  Eigen::Vector3d Rotate(const Eigen::Vector3d& vec) const { return q_ * vec; }

  // Spherical linear interpolation to `pose_to`, `t` belongs [0, 1]
  Pose3d Interpolate(const Pose3d& pose_to, double t) const {
    Eigen::Quaterniond q_interp = q_.slerp(t, pose_to.q_);
    Eigen::Vector3d p_interp = (pose_to.p_ - p_) * t + p_;
    return {q_interp, p_interp};
  }

  std::string ToString() const;

  Eigen::Quaterniond q() const { return q_; }

  Eigen::Vector3d p() const { return p_; }

 protected:
  Eigen::Quaterniond q_;  // orientation
  Eigen::Vector3d p_;     // position
};

Pose3d Interpolate(const Pose3d& pose_from, const Pose3d& pose_to, double t);

Pose3d operator*(const Pose3d& lhs, const Pose3d& rhs);

using Trans3d = Pose3d;

}  // namespace common
