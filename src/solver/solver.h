#pragma once

#include "ceres/ceres.h"

namespace oh_my_loam {

class Solver {
 public:
  Solver() = default;

  void SetInitialGuess(double* param_q, double* param_p);

 protected:
  std::unique_ptr<ceres::Problem> problem_;
  ceres::LocalParameterization* parameterization_;
  ceres::LossFunction* loss_function_;
};

}  // oh_my_loam