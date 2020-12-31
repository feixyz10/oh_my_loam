#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "common.h"
#include "utils.h"

namespace oh_my_loam {

struct VisFrame {
  double timestamp = std::nanf("");
  PointCloudPtr cloud = nullptr;
};

class Visualizer {
 public:
  Visualizer(const std::string &name, size_t max_history_size)
      : name_(name), max_history_size_(max_history_size) {
    // Start the thread to draw
    thread_.reset(new std::thread([&]() {
      viewer_.reset(new PCLVisualizer(name_));
      // Set background color
      const Color &bg_color = {0, 0, 0};
      viewer_->setBackgroundColor(bg_color.r, bg_color.g, bg_color.b);
      // Set camera position.
      viewer_->setCameraPosition(0, 0, 200, 0, 0, 0, 1, 0, 0, 0);
      viewer_->setSize(2500, 1500);
      viewer_->addCoordinateSystem(1.0);
      // Add mouse and keyboard callback.
      viewer_->registerKeyboardCallback(
          [&](const pcl::visualization::KeyboardEvent &event) -> void {
            KeyboardEventCallback(event);
          });
      Run();
    }));
  }

  virtual ~Visualizer() {
    is_stopped_ = true;
    viewer_->close();
    if (thread_->joinable()) {
      thread_->join();
    }
  }

  void Render(const std::shared_ptr<VisFrame> &frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    while (history_frames_.size() > max_history_size_) {
      history_frames_.pop_back();
    }
    history_frames_.emplace_front(frame);
    curr_frame_iter_ = history_frames_.begin();
    is_updated_ = true;
  }

  std::string Name() const { return name_; }

 protected:
  void Run() {
    while (!is_stopped_) {
      if (is_updated_) {
        RemoveRenderedObjects();
        Draw();
        is_updated_ = false;
      }
      viewer_->spinOnce(20);  // ms
    }
  }

  /**
   * @brief Draw objects, pure virtual function. Example code:
   * virtual void Draw() {
   *   auto frame = GetCurrentFrame<VisFrame>();
   *   AddPointCloud(frame.cloud, {255, 255, 255}, "point cloud");
   * }
   */
  virtual void Draw() = 0;

  /**
   * @brief Keyboard event callback function, This method should be overrided
   * for customization
   *
   * @param event Keyboard event
   */
  virtual void KeyboardEventCallback(
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
    } else if (event.getKeySym() == "r" && event.keyDown()) {
      viewer_->setCameraPosition(0, 0, 200, 0, 0, 0, 1, 0, 0, 0);
      viewer_->setSize(2500, 1500);
      is_updated_ = true;
    }
  }

  /**
   * @brief Remove all old rendered point clouds
   *
   */
  void RemoveRenderedObjects() {
    for (const auto &id : rendered_cloud_ids_) {
      viewer_->removePointCloud(id);
    }
    rendered_cloud_ids_.clear();
    viewer_->removeAllShapes();
  }

  template <typename FrameT>
  FrameT GetCurrentFrame() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return *static_cast<FrameT *>((*curr_frame_iter_).get());
  }

  template <typename PointType>
  void DrawPointCloud(
      const typename pcl::PointCloud<PointType>::ConstPtr &cloud,
      const Color &color, const std::string &id, int point_size = 3) {
    AddPointCloud<PointType>(cloud, color, id, viewer_.get(), point_size);
    rendered_cloud_ids_.push_back(id);
  }

  template <typename PointType>
  void DrawPointCloud(
      const typename pcl::PointCloud<PointType>::ConstPtr &cloud,
      const std::string &field, const std::string &id, int point_size = 3) {
    AddPointCloud<PointType>(cloud, field, id, viewer_.get(), point_size);
    rendered_cloud_ids_.push_back(id);
  }

  // visualizer name
  std::string name_;
  size_t max_history_size_;

  // atomic bool variable
  std::atomic_bool is_stopped_{false};

  // mutex for visualizer_frame.
  mutable std::mutex mutex_;

  // bool flag indicates whether the Visualizer frame is updated.
  std::atomic_bool is_updated_{false};

  // The visualizer frame list
  std::deque<std::shared_ptr<VisFrame>> history_frames_;

  // The current displayed frame iter.
  typename std::deque<std::shared_ptr<VisFrame>>::iterator curr_frame_iter_;

  // The rendered cloud ids.
  std::vector<std::string> rendered_cloud_ids_;

  // thread for visualization
  std::unique_ptr<std::thread> thread_ = nullptr;

  // viewer
  std::unique_ptr<PCLVisualizer> viewer_ = nullptr;
};

}  // namespace oh_my_loam