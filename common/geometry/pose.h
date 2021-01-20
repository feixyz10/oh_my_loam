#pragma once

#include <eigen3/Eigen/Dense>

namespace common {

template <typename T>
class Pose3 {
 public:
  Pose3() {
    r_quat_.setIdentity();
    t_vec_.setZero();
  };

  Pose3(const Eigen::Quaterniond& r_quat, const Eigen::Matrix<T, 3, 1>& t_vec)
      : r_quat_(q), t_vec_(p) {}

  Pose3(const Eigen::Matrix3d& r_mat, const Eigen::Matrix<T, 3, 1>& t_vec)
      : r_quat_(r_mat), t_vec_(t_vec) {}

  Pose3(const double* const r_quat, const double* const t_vec)
      : r_quat_(r_quat), t_vec_(t_vec) {}

  Pose3 Inv() const {
    Eigen::Quaternion<T> r_inv = r_quat_.inverse();
    Eigen::Matrix<T, 3, 1> t_inv = -r_inv * t_vec_;
    return {r_inv, t_inv};
  }

  Eigen::Matrix<T, 3, 1> Transform(const Eigen::Matrix<T, 3, 1>& point) const {
    return r_quat_ * point + t_vec_;
  }

  Eigen::Matrix<T, 3, 1> operator*(const Eigen::Matrix<T, 3, 1>& point) const {
    return Transform(point);
  }

  Eigen::Matrix<T, 3, 1> Translate(const Eigen::Matrix<T, 3, 1>& vec) const {
    return vec + t_vec_;
  }

  Eigen::Matrix<T, 3, 1> Rotate(const Eigen::Matrix<T, 3, 1>& vec) const {
    return r_quat_ * vec;
  }

  // Spherical linear interpolation to `pose_to`
  Pose3 Interpolate(const Pose3& pose_to, double t) const {
    Pose3 pose_dst;
    Eigen::Quaternion<T> q_interp = r_quat_.slerp(t, pose_to.r_quat_);
    Eigen::Matrix<T, 3, 1> p_interp = (pose_to.t_vec_ - t_vec_) * t + t_vec_;
    return {q_interp, p_interp};
  }

  std::string ToString() const {
    std::ostringstream oss;
    oss << "[Pose3D] q = (" << q_.coeffs().transpose() << "), p = ("
        << p_.transpose() << ")";
    return oss.str();
  }

  Eigen::Quaternion<T> r_quat() const { return r_quat_; }

  Eigen::Matrix<T, 3, 1> t_vec() const { return t_vec_; }

 protected:
  Eigen::Quaternion<T> r_quat_;   // orientation/rotation
  Eigen::Matrix<T, 3, 1> t_vec_;  // positon/translation
};

template <typename T>
Pose3<T> Interpolate(const Pose3<T>& pose_from, const Pose3<T>& pose_to,
                     double t) {
  return pose_from.Interpolate(pose_to, t);
}

template <typename T>
Pose3<T> operator*(const Pose3<T>& lhs, const Pose3<T>& rhs) {
  return Pose3<T>(lhs.q() * rhs.q(), lhs.q() * rhs.p() + lhs.p());
}

using Pose3d = Pose3<double>;

using Pose3f = Pose3<float>;

}  // namespace common
