#pragma once

#include "base_visualizer.h"
#include "extractor/feature_points.h"

namespace oh_my_loam {

struct OdometryVisFrame : public VisFrame {
  FeaturePoints feature_pts;
};

class OdometryVisualizer : public Visualizer {
 public:
  explicit OdometryVisualizer(const std::string &name = "OdometryVisualizer",
                              size_t max_history_size = 10)
      : Visualizer(name, max_history_size) {}

 private:
  void Draw() override;
};

}  // namespace oh_my_loam
