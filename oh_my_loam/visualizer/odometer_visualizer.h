#pragma once

#include "common/visualizer/lidar_visualizer.h"
#include "helper/helper.h"

namespace oh_my_loam {

struct OdometerVisFrame : public VisFrame {
  TPointCloudPtr surf_pts;
  TPointCloudPtr corn_pts;
  std::vector<PointLinePair> pl_pairs;
  std::vector<PointPlanePair> pp_pairs;
  Pose3d pose_curr2last;
  Pose3d pose_curr2world;
};

class OdometryVisualizer : public Visualizer {
 public:
  explicit OdometryVisualizer(const std::string &name = "OdometryVisualizer",
                              size_t max_history_size = 10)
      : Visualizer(name, max_history_size) {}

 private:
  void Draw() override;
  std::deque<Pose3d> poses_;
};

}  // namespace oh_my_loam
