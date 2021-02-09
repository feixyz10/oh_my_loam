#pragma once

#include "common/geometry/trajectory.h"
#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

struct OhmyloamVisFrame : public common::LidarVisFrame {
  TPointCloudConstPtr cloud_map_corn;
  TPointCloudConstPtr cloud_map_surf;
  TPointCloudConstPtr cloud_corn;  // current
  TPointCloudConstPtr cloud_surf;  // current
  common::Pose3d pose_map;
};

class OhmyloamVisualizer : public common::LidarVisualizer {
 public:
  explicit OhmyloamVisualizer(const std::string &save_path,
                              const std::string &name = "OhmyloamVisualizer",
                              size_t max_history_size = 10)
      : common::LidarVisualizer(name, max_history_size) {
    save_path_ = save_path;
  }

 private:
  void Draw() override;

  void KeyboardEventCallback(
      const pcl::visualization::KeyboardEvent &event) override;

  common::Trajectory traj_;

  std::string save_path_;
};

}  // namespace oh_my_loam
