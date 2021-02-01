#pragma once

#include "common/visualizer/lidar_visualizer.h"
#include "oh_my_loam/solver/cost_function.h"

namespace oh_my_loam {

struct OdometerVisFrame : public common::LidarVisFrame {
  TPointCloudConstPtr cloud_surf;
  TPointCloudConstPtr cloud_corn;
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

  void DrawCorn(const common::Pose3d &pose,
                const std::vector<PointLinePair> &pairs);

  void DrawSurf(const common::Pose3d &pose,
                const std::vector<PointPlanePair> &pairs);

  void DrawTrajectory();

  void KeyboardEventCallback(
      const pcl::visualization::KeyboardEvent &event) override;

  bool trans_ = true;
  bool corn_connect_ = false;
  bool surf_connect_ = false;

  std::deque<common::Pose3d> poses_;
};

}  // namespace oh_my_loam
