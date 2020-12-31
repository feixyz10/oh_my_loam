#include "extractor_visualizer.h"

namespace oh_my_loam {

void ExtractorVisualizer::Draw() {
  auto frame = GetCurrentFrame<ExtractorVisFrame>();
  DrawPointCloud<Point>(frame.cloud, WHITE, "raw_point_cloud");
  DrawPointCloud<TPoint>(frame.feature_pts.less_flat_surf_pts, CYAN,
                         "less_flat_surf_pts");
  DrawPointCloud<TPoint>(frame.feature_pts.flat_surf_pts, BLUE,
                         "flat_surf_pts");
  DrawPointCloud<TPoint>(frame.feature_pts.less_sharp_corner_pts, PURPLE,
                         "less_sharp_corner_pts");
  DrawPointCloud<TPoint>(frame.feature_pts.sharp_corner_pts, RED,
                         "sharp_corner_pts");
};

}  // namespace oh_my_loam
