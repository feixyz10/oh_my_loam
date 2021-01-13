#include "extractor_VLP16.h"

#include "common/math/math_utils.h"

namespace oh_my_loam {

int ExtractorVLP16::GetScanID(const common::Point& pt) const {
  double theta =
      common::Rad2Degree(std::atan2(pt.z, std::hypot(pt.x, pt.y))) + 15.0;
  return static_cast<int>(std::round(theta / 2.0) + 1.e-5);
};

}  // namespace oh_my_loam