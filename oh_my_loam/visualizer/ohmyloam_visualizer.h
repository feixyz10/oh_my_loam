#pragma once

#include "common/geometry/trajectory.h"
#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

struct OhmyloamVisFrame : public common::LidarVisFrame {
  TPointCloudConstPtr cloud_map_corn;
  TPointCloudConstPtr cloud_map_surf;
  TPointCloudPtr cloud_corn;  // current
  TPointCloudPtr cloud_surf;  // current
  common::Pose3d pose_map;
};

class OhmyloamVisualizer : public common::LidarVisualizer {
 public:
  explicit OhmyloamVisualizer(const std::string &name = "OhmyloamVisualizer",
                              size_t max_history_size = 10)
      : common::LidarVisualizer(name, max_history_size) {}

 private:
  void Draw() override;

  common::Trajectory traj_;
};

}  // namespace oh_my_loam
