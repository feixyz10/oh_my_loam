#pragma once

#include "feature_extractor_base.h"
#include "visualizer_base.h"

namespace oh_my_loam {

struct FeaturePointsVisFrame : public VisFrame {
  std::shared_ptr<const FeaturePoints> feature_pts;
};

class FeaturePointsVisualizer : public Visualizer<FeaturePointsVisFrame> {
 public:
  explicit FeaturePointsVisualizer(
      const std::string &name = "FeaturePointsVisFrame",
      size_t max_history_size = 10)
      : Visualizer<FeaturePointsVisFrame>(name, max_history_size) {}

 protected:
  void Draw() override final {
    auto frame = GetCurrentFrame();
    {  // add raw point cloud
      std::string id = "raw point cloud";
      DrawPointCloud<Point>(*frame.cloud, WHITE, id, viewer_.get());
      rendered_cloud_ids_.push_back(id);
    }
    {  // add all feature_pts
      std::string id = "feature_pts";
      DrawPointCloud<IPoint>(frame.feature_pts->feature_pts, GRAY, id,
                             viewer_.get());
      rendered_cloud_ids_.push_back(id);
    }
    // {  // add flat_surf_pts
    //   std::string id = "flat_surf_pts";
    //   DrawPointCloud(*frame.feature_ptsflat_surf_pts, CYAN, id,
    //   viewer_.get());
    //   rendered_cloud_ids_.push_back(id);
    // }
    // {  // add less_flat_surf_pts
    //   std::string id = "less_flat_surf_pts";
    //   DrawPointCloud(*frame.feature_ptsless_flat_surf_pts, GREEN, id,
    //   viewer_.get());
    //   rendered_cloud_ids_.push_back(id);
    // }
    // {  // add less_sharp_corner_pts
    //   std::string id = "less_sharp_corner_pts";
    //   DrawPointCloud(*frame.feature_ptsless_sharp_corner_pts, ORANGE, id,
    //   viewer_.get());
    //   rendered_cloud_ids_.push_back(id);
    // }
    // {  // add sharp_corner_pts
    //   std::string id = "sharp_corner_pts";
    //   DrawPointCloud(*frame.feature_pts.sharp_corner_pts, ORANGE, id,
    //   viewer_.get());
    //   rendered_cloud_ids_.push_back(id);
    // }
  }
};

}  // oh_my_loam
