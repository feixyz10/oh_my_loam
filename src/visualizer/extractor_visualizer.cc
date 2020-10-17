#include "extractor_visualizer.h"

namespace oh_my_loam {

void ExtractorVisualizer::Draw() {
  auto frame = GetCurrentFrame();
  {  // add raw point cloud
    std::string id = "raw point cloud";
    DrawPointCloud(*frame.cloud, WHITE, id, viewer_.get());
    rendered_cloud_ids_.push_back(id);
  }
  {  // add less_flat_surf_pts
    std::string id = "less_flat_surf_pts";
    DrawPointCloud(*frame.feature_pts.less_flat_surf_pts, CYAN, id,
                   viewer_.get(), 5);
    rendered_cloud_ids_.push_back(id);
  }
  {  // add flat_surf_pts
    std::string id = "flat_surf_pts";
    DrawPointCloud(*frame.feature_pts.flat_surf_pts, BLUE, id, viewer_.get(),
                   7);
    rendered_cloud_ids_.push_back(id);
  }
  {  // add less_sharp_corner_pts
    std::string id = "less_sharp_corner_pts";
    DrawPointCloud(*frame.feature_pts.less_sharp_corner_pts, PURPLE, id,
                   viewer_.get(), 5);
    rendered_cloud_ids_.push_back(id);
  }
  {  // add sharp_corner_pts
    std::string id = "sharp_corner_pts";
    DrawPointCloud(*frame.feature_pts.sharp_corner_pts, RED, id, viewer_.get(),
                   7);
    rendered_cloud_ids_.push_back(id);
  }
};

}  // namespace oh_my_loam
