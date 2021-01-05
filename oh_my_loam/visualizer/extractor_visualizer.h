#pragma once

#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/base/feature.h"

namespace oh_my_loam {

struct ExtractorVisFrame : public common::LidarVisFrame {
  Feature feature;
};

class ExtractorVisualizer : public common::LidarVisualizer {
 public:
  explicit ExtractorVisualizer(const std::string &name = "ExtractorVisualizer",
                               size_t max_history_size = 10)
      : common::LidarVisualizer(name, max_history_size) {}

 private:
  void Draw() override;
};

}  // namespace oh_my_loam
