#pragma once

#include <yaml-cpp/yaml.h>

#include "common.h"
#include "extractor/base_extractor.h"

namespace oh_my_loam {

class OhMyLoam {
 public:
  OhMyLoam() = default;

  bool Init();

  void Run(const PointCloud& cloud_in, double timestamp);

 private:
  std::unique_ptr<Extractor> extractor_{nullptr};

  // remove outliers: nan and very close points
  void RemoveOutliers(const PointCloud& cloud_in,
                      PointCloud* const cloud_out) const;

  DISALLOW_COPY_AND_ASSIGN(OhMyLoam)
};

}  // namespace oh_my_loam