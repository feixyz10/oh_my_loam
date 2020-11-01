#include "extractor_VLP16.h"

namespace oh_my_loam {

int ExtractorVLP16::GetScanID(const Point& pt) const {
  double omega = std::atan2(pt.z, Distance(pt)) * 180 * M_1_PI + 15.0;
  // static int i = 0;
  // if (i++ < 10) {
  //   ADEBUG << "OMEGA: " << std::atan2(pt.z, Distance(pt)) * 180 * M_1_PI
  //   + 15.0
  //          << " id = " << static_cast<int>(std::round(omega / 2.0) + 0.01)
  //          << " z = " << pt.z << " "
  //          << " d = " << Distance(pt) << std::endl;
  // }
  return static_cast<int>(std::round(omega / 2.0) + 1.e-5);
};

}  // namespace oh_my_loam