#include "pose.h"

namespace common {

Pose3d Interpolate(const Pose3d& pose_from, const Pose3d& pose_to, double t) {
  return pose_from.Interpolate(pose_to, t);
}

Pose3d operator*(const Pose3d& lhs, const Pose3d& rhs) {
  return Pose3d(lhs.q() * rhs.q(), lhs.q() * rhs.p() + lhs.p());
}

std::string Pose3d::ToString() const {
  std::ostringstream oss;
  oss << "[Pose3d] q = (" << q_.coeffs().transpose() << "), p = ("
      << p_.transpose() << ")";
  return oss.str();
}

}  // namespace common
