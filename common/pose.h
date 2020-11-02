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

  Pose3D(const Eigen::Matrix3d& r_mat, const Eigen::Vector3d& p)
      : q_(r_mat), p_(p) {}

  Pose3D(const double* const q, const double* const p) : q_(q), p_(p) {}

  Pose3D Inv() const {
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

  template <typename PointT>
  PointT Transform(const PointT& pt) const {
    PointT pt_n = pt;
    Eigen::Vector3d vec = Transform(Eigen::Vector3d(pt.x, pt.y, pt.z));
    pt_n.x = static_cast<float>(vec.x());
    pt_n.y = static_cast<float>(vec.y());
    pt_n.z = static_cast<float>(vec.z());
    return pt_n;
  }

  template <typename PointT>
  PointT operator*(const PointT& vec) const {
    return Transform<PointT>(vec);
  }

  Eigen::Vector3d Translate(const Eigen::Vector3d& vec) const {
    return vec + p_;
  }

  Eigen::Vector3d Rotate(const Eigen::Vector3d& vec) const { return q_ * vec; }

  // Spherical linear interpolation to `pose_to`, `t` belongs [0, 1]
  Pose3D Interpolate(const Pose3D& pose_to, double t) const {
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

Pose3D Interpolate(const Pose3D& pose_from, const Pose3D& pose_to, double t);

Pose3D operator*(const Pose3D& lhs, const Pose3D& rhs);

using Trans3D = Pose3D;

}  // namespace oh_my_loam
