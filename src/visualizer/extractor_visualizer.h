#pragma once

#include "base_visualizer.h"
#include "extractor/feature_points.h"

namespace oh_my_loam {

struct ExtractorVisFrame : public VisFrame {
  FeaturePoints feature_pts;
};

class ExtractorVisualizer : public Visualizer<ExtractorVisFrame> {
 public:
  explicit ExtractorVisualizer(const std::string &name = "ExtractorVisualizer",
                               size_t max_history_size = 10)
      : Visualizer<ExtractorVisFrame>(name, max_history_size) {}

 private:
  void Draw() override;
};

}  // namespace oh_my_loam
