#pragma once

#include "common/geometry/pose3d.h"
#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

struct OhmyloamVisFrame : public common::LidarVisFrame {
  TPointCloudConstPtr cloud_map_corn;
  TPointCloudConstPtr cloud_map_surf;
  TPointCloudConstPtr cloud_corn;  // current
  TPointCloudConstPtr cloud_surf;  // current
  common::Pose3d pose_odom;
  common::Pose3d pose_map;
};

class OhmyloamVisualizer : public common::LidarVisualizer {
 public:
  explicit OhmyloamVisualizer(const std::string &name = "OhmyloamVisualizer",
                              size_t max_history_size = 10)
      : common::LidarVisualizer(name, max_history_size) {}

 private:
  void Draw() override;

  void DrawTrajectory(const std::vector<common::Pose3d> &poses,
                      const std::string &id, const common::Color &color);

  void KeyboardEventCallback(
      const pcl::visualization::KeyboardEvent &event) override;

  std::vector<common::Pose3d> poses_odom_;
  std::vector<common::Pose3d> poses_map_;
  bool trans_ = true;
};

}  // namespace oh_my_loam
