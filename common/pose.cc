#include <pose.h>

namespace oh_my_loam {

Pose3D Interpolate(const Pose3D& pose_from, const Pose3D& pose_to, double t) {
  return pose_from.Interpolate(pose_to, t);
}

Pose3D operator*(const Pose3D& lhs, const Pose3D& rhs) {
  return Pose3D(lhs.q() * rhs.q(), lhs.q() * rhs.p() + lhs.p());
}

std::string Pose3D::ToString() const {
  std::ostringstream oss;
  oss << "[Pose3D] q = (" << q_.coeffs().transpose() << "), p = ("
      << p_.transpose() << ")";
  return oss.str();
}

}  // namespace oh_my_loam
