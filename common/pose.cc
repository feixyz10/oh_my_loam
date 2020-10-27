#include <pose.h>

namespace oh_my_loam {

Pose3D Interpolate(const Pose3D& pose_from, const Pose3D& pose_to, double t) {
  return pose_from.InterPolate(pose_to, t);
}

}  // namespace oh_my_loam
