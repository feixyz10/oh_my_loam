#pragma once

#include "feature_extractor_base.h"
#include "utils.h"

namespace oh_my_loam {

// for VLP-16
class FeatureExtractorVLP16 : public FeaturePointsExtractor {
 public:
  FeatureExtractorVLP16() { num_scans_ = 16; }

 protected:
  int GetScanID(const Point& pt) const override final {
    static int i = 0;
    double omega = std::atan2(pt.z, Distance(pt)) * 180 * M_1_PI + 15.0;
    if (i++ < 10)
      std::cout << "OMEGA: "
                << std::atan2(pt.z, Distance(pt)) * 180 * M_1_PI + 15.0
                << " id = " << static_cast<int>(std::round(omega / 2.0) + 0.01)
                << " z = " << pt.z << " "
                << " d = " << Distance(pt) << std::endl;
    return static_cast<int>(std::round(omega / 2.0) + 1.e-5);
  }
};

}  // namespace oh_my_loam