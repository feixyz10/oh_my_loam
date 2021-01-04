#include "extractor_VLP16.h"

namespace oh_my_loam {

int ExtractorVLP16::GetScanID(const Point& pt) const {
  double omega = std::atan2(pt.z, std::hypot(pt.x, pt.y)) * 180 * M_1_PI + 15.0;
  return static_cast<int>(std::round(omega / 2.0) + 1.e-5);
};

}  // namespace oh_my_loam