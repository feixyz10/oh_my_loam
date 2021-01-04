#pragma once

#include "cost_function.h"

namespace oh_my_loam {

class PoseSolver {
 public:
  PoseSolver(double* q, double* p);

  void AddPointLinePair(const PointLinePair& pair, double time);

  void AddPointPlanePair(const PointPlanePair& pair, double time);

  void Solve(int max_iter_num = 5, bool verbose = false);

 private:
  ceres::Problem problem_;

  ceres::LossFunction* loss_function_;

  double *q_, *p_;

  DISALLOW_COPY_AND_ASSIGN(PoseSolver)
};

}  // oh_my_loam