#pragma once

#include "base_feature_extractor.h"
#include "utils.h"

namespace oh_loam {

// for VLP-16
class FeatureExtractorVLP16 : public FeatureExtractor {
 public:
  FeatureExtractorVLP16() { num_scans_ = 16; }

 private:
  int GetScanID(const Point& pt) const override final {
    double omega = std::atan2(pt.z, Distance(pt)) * 180 * M_1_PI + 15.0;
    return static_cast<int>(std::round(omega) + 0.01);
  }
}

}  // oh_loam