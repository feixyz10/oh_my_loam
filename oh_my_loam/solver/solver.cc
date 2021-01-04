#include "solver.h"

namespace oh_my_loam {

namespace {
double kHuberLossScale = 0.1;
}

PoseSolver::PoseSolver(double *q, double *p) : q_(q), p_(p) {
  loss_function_ = new ceres::HuberLoss(kHuberLossScale);
  problem_.AddParameterBlock(q_, 4,
                             new ceres::EigenQuaternionParameterization());
  problem_.AddParameterBlock(p_, 3);
}

void PoseSolver::AddPointLinePair(const PointLinePair &pair, double time) {
  ceres::CostFunction *cost_function =
      PointLineCostFunction::Create(pair, time);
  problem_.AddResidualBlock(cost_function, loss_function_, q_, p_);
}

void PoseSolver::AddPointPlanePair(const PointPlanePair &pair, double time) {
  ceres::CostFunction *cost_function =
      PointPlaneCostFunction::Create(pair, time);
  problem_.AddResidualBlock(cost_function, loss_function_, q_, p_);
}

void PoseSolver::Solve(int max_iter_num, bool verbose) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = max_iter_num;
  options.minimizer_progress_to_stdout = verbose;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  AINFO_IF(verbose) << summary.BriefReport();
}

}  // oh_my_loam