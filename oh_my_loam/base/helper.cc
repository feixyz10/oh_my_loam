#include "helper.h"

namespace oh_my_loam {

void TransformToStart(const Pose3d& pose, const TPoint& pt_in,
                      TPoint* const pt_out) {
  Pose3d pose_interp = Pose3d().Interpolate(pose, GetTime(pt_in));
  TransformPoint<TPoint>(pose_interp, pt_in, pt_out);
}

void TransformToEnd(const Pose3d& pose, const TPoint& pt_in,
                    TPoint* const pt_out) {
  TransformToStart(pose, pt_in, pt_out);
  TransformPoint<TPoint>(pose.Inv(), *pt_out, pt_out);
}

}  // namespace oh_my_loam