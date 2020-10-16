#pragma once

#include <yaml-cpp/yaml.h>

#include "common.h"
#include "feature_points_extractor/base_feature_points_extractor.h"

namespace oh_my_loam {

class OhMyLoam {
 public:
  OhMyLoam() = default;

  bool Init();

  void Run(const PointCloud& cloud, double timestamp);

 private:
  std::unique_ptr<FeaturePointsExtractor> feature_extractor_{nullptr};

  DISALLOW_COPY_AND_ASSIGN(OhMyLoam)
};

}  // namespace oh_my_loam