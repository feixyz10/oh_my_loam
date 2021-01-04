#include "math_utils.h"

#include "common/log/log.h"

namespace common {

double NormalizeAngle(double ang) {
  const double& two_pi = 2 * M_PI;
  return ang - two_pi * std::floor((ang + M_PI) / two_pi);
}

const std::vector<int> Range(int begin, int end, int step) {
  ACHECK(step != 0) << "Step must non-zero";
  int num = (end - begin) / step;
  if (num <= 0) return {};
  end = begin + step * num;
  std::vector<int> seq(num);
  for (int i = begin; i != end; i += step) seq[i] = i;
  return seq;
}

const std::vector<int> Range(int end) { return Range(0, end, 1); }

}  // namespace common