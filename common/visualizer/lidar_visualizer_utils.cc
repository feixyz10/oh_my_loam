#include "lidar_visualizer_utils.h"

namespace common {

void AddTrajectory(const Trajectory &trajectory, const Color &color,
                   const std::string &id, PCLVisualizer *const viewer) {
  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    Eigen::Vector3f p1 = trajectory.at(i).t_vec().cast<float>();
    Eigen::Vector3f p2 = trajectory.at(i + 1).t_vec().cast<float>();
    AddLine<Point>({p1.x(), p1.y(), p1.z()}, {p2.x(), p2.y(), p2.z()}, color,
                   id + std::to_string(i), viewer);
  }
}

}  // namespace common