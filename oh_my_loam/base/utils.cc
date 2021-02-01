#include "oh_my_loam/base/utils.h"

#include "common/pcl/pcl_utils.h"

namespace oh_my_loam {

void TransformToStart(const common::Pose3d &pose, const TPoint &pt_in,
                      TPoint *const pt_out) {
  common::Pose3d pose_interp =
      common::Pose3d().Interpolate(pose, GetTime(pt_in));
  common::TransformPoint<TPoint>(pose_interp, pt_in, pt_out);
}

TPoint TransformToStart(const common::Pose3d &pose, const TPoint &pt_in) {
  TPoint pt_out;
  TransformToStart(pose, pt_in, &pt_out);
  return pt_out;
}

void TransformToEnd(const common::Pose3d &pose, const TPoint &pt_in,
                    TPoint *const pt_out) {
  TransformToStart(pose, pt_in, pt_out);
  common::TransformPoint<TPoint>(pose.Inv(), *pt_out, pt_out);
}

TPoint TransformToEnd(const common::Pose3d &pose, const TPoint &pt_in) {
  TPoint pt_out;
  TransformToEnd(pose, pt_in, &pt_out);
  return pt_out;
}

}  // namespace oh_my_loam