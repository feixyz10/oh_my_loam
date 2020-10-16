#pragma once

#include "base_visualizer.h"
#include "feature_points_extractor/feature_points.h"

namespace oh_my_loam {

struct FeaturePointsVisFrame : public VisFrame {
  FeaturePoints feature_pts;
};

class FeaturePointsVisualizer : public Visualizer<FeaturePointsVisFrame> {
 public:
  explicit FeaturePointsVisualizer(
      const std::string &name = "FeaturePointsVisualizer",
      size_t max_history_size = 10)
      : Visualizer<FeaturePointsVisFrame>(name, max_history_size) {}

 private:
  void Draw() override;
};

}  // namespace oh_my_loam
