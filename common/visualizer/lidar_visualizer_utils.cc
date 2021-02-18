#include "lidar_visualizer_utils.h"

namespace common {

void AddTrajectory(const Trajectory &trajectory, const Color &color,
                   const std::string &id, PCLVisualizer *const viewer) {
  const auto &points = trajectory.GetPointSeq();
  PointCloudPtr cloud(new PointCloud);
  for (const auto &p : points) cloud->points.emplace_back(p.x(), p.y(), p.z());
  AddPointCloud<Point>(cloud, color, id, viewer, 5);
}

}  // namespace common