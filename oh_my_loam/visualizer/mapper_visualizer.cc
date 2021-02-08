#include "mapper_visualizer.h"

#include "common/color/color.h"

namespace oh_my_loam {

namespace {
using namespace common;
}  // namespace

void MapperVisualizer::Draw() {
  auto frame = GetCurrentFrame<MapperVisFrame>();
  DrawPointCloud<TPoint>(frame.cloud_corn, GRAY, "cloud_corn");
  DrawPointCloud<TPoint>(frame.cloud_surf, GRAY, "cloud_surf");

  DrawCorn(frame.pose_curr2odom, frame.pose_curr2map, frame.pl_pairs);
  DrawSurf(frame.pose_curr2odom, frame.pose_curr2map, frame.pp_pairs);

  traj_odom_.AddPose(frame.pose_curr2odom);
  traj_map_.AddPose(frame.pose_curr2map);
  DrawTrajectory();
}

void MapperVisualizer::DrawCorn(const common::Pose3d &pose_odom,
                                const common::Pose3d &pose_map,
                                const std::vector<PointLineCoeffPair> &pairs) {
  TPointCloudPtr src(new TPointCloud);
  src->resize(pairs.size());
  const auto &pose = trans_ ? pose_map : pose_odom;
  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto &pair = pairs[i];
    TransformToStart(pose, pair.pt, &src->at(i));
  }
  DrawPointCloud<TPoint>(src, RED, "src_corn", 4);
}

void MapperVisualizer::DrawSurf(const common::Pose3d &pose_odom,
                                const common::Pose3d &pose_map,
                                const std::vector<PointPlaneCoeffPair> &pairs) {
  TPointCloudPtr src(new TPointCloud);
  src->resize(pairs.size());
  const auto &pose = trans_ ? pose_map : pose_odom;
  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto &pair = pairs[i];
    TransformToStart(pose, pair.pt, &src->at(i));
  }
  DrawPointCloud<TPoint>(src, CYAN, "src_surf", 4);
}

void MapperVisualizer::DrawTrajectory() {
  common::AddTrajectory(traj_odom_, PINK, "traj_odmo", viewer_.get());
  common::AddTrajectory(traj_map_, PURPLE, "traj_map", viewer_.get());
}

void MapperVisualizer::KeyboardEventCallback(
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
