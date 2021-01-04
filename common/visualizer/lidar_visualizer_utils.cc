#include "lidar_visualizer_utils.h"

namespace common {

void AddLine(const pcl::PointXYZ& pt1, const pcl::PointXYZ& pt2,
             const Color& color, const std::string& id,
             PCLVisualizer* const viewer) {
  viewer->addLine(pt1, pt2, color.r, color.g, color.b, id);
}

void DrawSphere(const pcl::PointXYZ& center, double radius, const Color& color,
                const std::string& id, PCLVisualizer* const viewer) {
  viewer->addSphere(center, radius, color.r, color.g, color.b, id);
}

}  // namespace common