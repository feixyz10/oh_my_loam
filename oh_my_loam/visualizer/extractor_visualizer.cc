#include "extractor_visualizer.h"

namespace oh_my_loam {
namespace {
using namespace common;
}  // namespace

void ExtractorVisualizer::Draw() {
  auto frame = GetCurrentFrame<ExtractorVisFrame>();
  DrawPointCloud<Point>(frame.cloud, GRAY, "cloud_raw");
  for (size_t i = 0; i < frame.features.size(); ++i) {
    const auto &feature = frame.features[i];
    std::string id_suffix = std::to_string(i);
    DrawPointCloud<TPoint>(feature.cloud_surf, BLUE, "cloud_surf" + id_suffix);
    DrawPointCloud<TPoint>(feature.cloud_flat_surf, CYAN,
                           "cloud_flat_surf" + id_suffix);
    DrawPointCloud<TPoint>(feature.cloud_corner, YELLOW,
                           "cloud_corner" + id_suffix);
    DrawPointCloud<TPoint>(feature.cloud_sharp_corner, RED,
                           "cloud_sharp_corner" + id_suffix);
  }
};

}  // namespace oh_my_loam
