#include "utils.h"

namespace oh_my_loam {

double NormalizeAngle(double ang) {
  const double& two_pi = 2 * M_PI;
  return ang - two_pi * std::floor((ang + M_PI) / two_pi);
}

}  // oh_my_loam