#include "solver.h"

#include <Eigen/src/Core/Matrix.h>
#include <ceres/loss_function.h>
#include <ceres/types.h>

#include "common/log/log.h"

namespace oh_my_loam {

namespace {
double kHuberLossScale = 0.1;
}

PoseSolver::PoseSolver(const common::Pose3d &pose) {
  std::copy_n(pose.r_quat().coeffs().data(), 4, r_quat_);
  std::copy_n(pose.t_vec().data(), 3, t_vec_);
  loss_function_ = new ceres::HuberLoss(kHuberLossScale);
  problem_.AddParameterBlock(r_quat_, 4,
                             new ceres::EigenQuaternionParameterization());
  problem_.AddParameterBlock(t_vec_, 3);
}

bool PoseSolver::Solve(int max_iter_num, bool verbose,
                       common::Pose3d *const pose) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = max_iter_num;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  AINFO_IF(verbose) << summary.BriefReport();
  if (pose) *pose = common::Pose3d(r_quat_, t_vec_);
  return summary.termination_type == ceres::CONVERGENCE;
}

void PoseSolver::AddPointLinePair(const PointLinePair &pair, double time) {
  ceres::CostFunction *cost_function =
      PointLineCostFunction::Create(pair, time);
  problem_.AddResidualBlock(cost_function, loss_function_, r_quat_, t_vec_);
}

void PoseSolver::AddPointPlanePair(const PointPlanePair &pair, double time) {
  ceres::CostFunction *cost_function =
      PointPlaneCostFunction::Create(pair, time);
  problem_.AddResidualBlock(cost_function, loss_function_, r_quat_, t_vec_);
}

common::Pose3d PoseSolver::GetPose() const {
  return common::Pose3d(r_quat_, t_vec_);
}

}  // namespace oh_my_loam