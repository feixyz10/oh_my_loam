#pragma once

#include "common/geometry/pose3d.h"
#include "cost_function.h"

namespace oh_my_loam {

class PoseSolver {
public:
  PoseSolver(const common::Pose3d &pose);

  void AddPointLinePair(const PointLinePair &pair, double time);

  void AddPointPlanePair(const PointPlanePair &pair, double time);

  double Solve(int max_iter_num = 5, bool verbose = false,
               common::Pose3d *const pose = nullptr);

  common::Pose3d GetPose() const;

private:
  ceres::Problem problem_;

  ceres::LossFunction *loss_function_;

  // r_quat_: [x, y, z, w], t_vec_: [x, y, z]
  double *r_quat_, *t_vec_;

  DISALLOW_COPY_AND_ASSIGN(PoseSolver)
};

} // namespace oh_my_loam