#include "extractor_visualizer.h"

namespace oh_my_loam {
namespace {
using namespace common;
}  // namespace

void ExtractorVisualizer::Draw() {
  auto frame = GetCurrentFrame<ExtractorVisFrame>();
  DrawPointCloud<Point>(frame.cloud, WHITE, "cloud_raw");
  DrawPointCloud<TPoint>(frame.feature.cloud_less_flat_surf, CYAN,
                         "cloud_less_flat_surf");
  DrawPointCloud<TPoint>(frame.feature.cloud_flat_surf, BLUE,
                         "cloud_flat_surf");
  DrawPointCloud<TPoint>(frame.feature.cloud_less_sharp_corner, PURPLE,
                         "cloud_less_sharp_corner");
  DrawPointCloud<TPoint>(frame.feature.cloud_sharp_corner, RED,
                         "cloud_sharp_corner");
};

}  // namespace oh_my_loam
