#include "ohmyloam_visualizer.h"

#include "common/color/color.h"
#include "common/pcl/pcl_utils.h"

namespace oh_my_loam {

namespace {
using namespace common;
#define LIGHT_BLUE common::Color(0, 0, 150)
#define LIGHT_YELLOW common::Color(150, 150, 0)
}  // namespace

void OhmyloamVisualizer::Draw() {
  auto frame = GetCurrentFrame<OhmyloamVisFrame>();
  DrawPointCloud<TPoint>(frame.cloud_map_corn, GRAY, "cloud_map_corn");
  DrawPointCloud<TPoint>(frame.cloud_map_surf, GRAY, "cloud_map_surf");
  TransformPointCloud(frame.pose_map, *frame.cloud_corn,
                      frame.cloud_corn.get());
  DrawPointCloud<TPoint>(frame.cloud_corn, "time", "cloud_corn");
  TransformPointCloud(frame.pose_map, *frame.cloud_surf,
                      frame.cloud_surf.get());
  DrawPointCloud<TPoint>(frame.cloud_surf, "time", "cloud_surf");
  traj_.AddPose(frame.pose_map);
  AddTrajectory(traj_, PINK, "trajectory", viewer_.get());
}

}  // namespace oh_my_loam