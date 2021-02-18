#include "ohmyloam_visualizer.h"

#include <pcl/io/pcd_io.h>
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
  DrawPointCloud<TPoint>(frame.cloud_map_corn, GRAY, "cloud_map_corn", 1);
  DrawPointCloud<TPoint>(frame.cloud_map_surf, GRAY, "cloud_map_surf", 1);
  TPointCloudPtr cloud_corn_trans(new TPointCloud);
  TransformPointCloud(frame.pose_map, *frame.cloud_corn,
                      cloud_corn_trans.get());
  DrawPointCloud<TPoint>(cloud_corn_trans, "time", "cloud_corn");
  TPointCloudPtr cloud_surf_trans(new TPointCloud);
  TransformPointCloud(frame.pose_map, *frame.cloud_surf,
                      cloud_surf_trans.get());
  DrawPointCloud<TPoint>(cloud_surf_trans, "time", "cloud_surf");
  traj_.AddPose(frame.pose_map);
  AddTrajectory(traj_, VIOLET, "trajectory", viewer_.get());
}

void OhmyloamVisualizer::KeyboardEventCallback(
    const pcl::visualization::KeyboardEvent &event) {
  if (event.getKeySym() == "p" && event.keyDown()) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++curr_frame_iter_;
    if (curr_frame_iter_ == history_frames_.end()) {
      --curr_frame_iter_;
    }
    is_updated_ = true;
  } else if (event.getKeySym() == "n" && event.keyDown()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (curr_frame_iter_ != history_frames_.begin()) {
      --curr_frame_iter_;
    }
    is_updated_ = true;
  } else if (event.getKeySym() == "s" && event.keyDown()) {
    auto frame = GetCurrentFrame<OhmyloamVisFrame>();
    PointCloud cloud;
    cloud.reserve(frame.cloud_map_surf->size() + frame.cloud_map_surf->size());
    for (const auto &p : *frame.cloud_map_corn) {
      cloud.push_back({p.x, p.y, p.z});
    }
    for (const auto p : *frame.cloud_map_surf) {
      cloud.push_back({p.x, p.y, p.z});
    }
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = false;
    pcl::io::savePCDFileASCII(save_path_, cloud);

  } else if (event.getKeySym() == "r" && event.keyDown()) {
    viewer_->setCameraPosition(0, 0, 200, 0, 0, 0, 1, 0, 0, 0);
    viewer_->setSize(2500, 1500);
    is_updated_ = true;
  }
}

}  // namespace oh_my_loam