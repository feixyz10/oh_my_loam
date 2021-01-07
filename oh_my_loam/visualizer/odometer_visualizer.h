#pragma once

#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/base/helper.h"

namespace oh_my_loam {

struct OdometerVisFrame : public common::LidarVisFrame {
  common::TPointCloudPtr surf_pts;
  common::TPointCloudPtr corn_pts;
  std::vector<PointLinePair> pl_pairs;
  std::vector<PointPlanePair> pp_pairs;
  common::Pose3D pose_curr2last;
  common::Pose3D pose_curr2world;
};

class OdometerVisualizer : public common::LidarVisualizer {
 public:
  explicit OdometerVisualizer(const std::string &name = "OdometerVisualizer",
                              size_t max_history_size = 10)
      : common::LidarVisualizer(name, max_history_size) {}

 private:
  void Draw() override;

  std::deque<common::Pose3D> poses_;
};

}  // namespace oh_my_loam
