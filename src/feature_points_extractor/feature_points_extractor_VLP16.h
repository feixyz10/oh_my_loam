#pragma once

#include "base_feature_points_extractor.h"

namespace oh_my_loam {

// for VLP-16
class FeaturePointsExtractorVLP16 : public FeaturePointsExtractor {
 public:
  FeaturePointsExtractorVLP16() { num_scans_ = 16; }

 private:
  int GetScanID(const Point& pt) const override;
};

}  // namespace oh_my_loam