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
  if (!trans_) {
    DrawPointCloud<TPoint>(frame.cloud_corn, RED, "cloud_corn");
    DrawPointCloud<TPoint>(frame.cloud_surf, CYAN, "cloud_surf");
  } else {
    TPointCloudPtr cloud_corn_trans(new TPointCloud);
    TPointCloudPtr cloud_surf_trans(new TPointCloud);
    common::TransformPointCloud(frame.pose_map, *frame.cloud_corn,
                                cloud_corn_trans.get());
    common::TransformPointCloud(frame.pose_map, *frame.cloud_surf,
                                cloud_surf_trans.get());
    DrawPointCloud<TPoint>(cloud_corn_trans, RED, "cloud_corn");
    DrawPointCloud<TPoint>(cloud_surf_trans, CYAN, "cloud_surf");
  }
  // poses_odom_.push_back(frame.pose_odom);
  // DrawTrajectory(poses_odom_, "odom_traj", PINK);
  poses_map_.push_back(frame.pose_map);
  DrawTrajectory(poses_map_, "map_traj", WHITE);
}

void OhmyloamVisualizer::DrawTrajectory(
    const std::vector<common::Pose3d> &poses, const std::string &id,
    const common::Color &color) {
  for (size_t i = 0; i < poses.size() - 1; ++i) {
    Eigen::Vector3f p1 = poses[i].t_vec().cast<float>();
    Eigen::Vector3f p2 = poses[i + 1].t_vec().cast<float>();
    AddLine<Point>({p1.x(), p1.y(), p1.z()}, {p2.x(), p2.y(), p2.z()}, color,
                   id + std::to_string(i), viewer_.get());
  }
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
  } else if (event.getKeySym() == "t" && event.keyDown()) {
    trans_ = !trans_;
    is_updated_ = true;
  } else if (event.getKeySym() == "r" && event.keyDown()) {
    viewer_->setCameraPosition(0, 0, 200, 0, 0, 0, 1, 0, 0, 0);
    viewer_->setSize(2500, 1500);
    trans_ = true;
    is_updated_ = true;
  }
}

}  // namespace oh_my_loam