#pragma once

#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/base/helper.h"

namespace oh_my_loam {

struct OdometerVisFrame : public common::LidarVisFrame {
  TPointCloudPtr cloud_surf;
  TPointCloudPtr cloud_corner;
  std::vector<PointLinePair> pl_pairs;
  std::vector<PointPlanePair> pp_pairs;
  common::Pose3d pose_curr2last;
  common::Pose3d pose_curr2world;
};

class OdometerVisualizer : public common::LidarVisualizer {
 public:
  explicit OdometerVisualizer(const std::string &name = "OdometerVisualizer",
                              size_t max_history_size = 10)
      : common::LidarVisualizer(name, max_history_size) {}

 private:
  void Draw() override;

  std::deque<common::Pose3d> poses_;
};

}  // namespace oh_my_loam
