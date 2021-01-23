#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

#include "common/macro/macros.h"
#include "lidar_visualizer_utils.h"

namespace common {

struct LidarVisFrame {
  double timestamp = 0.0;
  PointCloudConstPtr cloud = nullptr;
};

class LidarVisualizer {
 public:
  explicit LidarVisualizer(const std::string &name,
                           size_t max_history_size = 10)
      : name_(name), max_history_size_(max_history_size) {
    thread_.reset(new std::thread([&]() {
      viewer_.reset(new PCLVisualizer(name_));
      // Set background color: black
      viewer_->setBackgroundColor(0, 0, 0);
      // Set camera position
      viewer_->setCameraPosition(0, 0, 200, 0, 0, 0, 1, 0, 0, 0);
      viewer_->setSize(2500, 1500);
      viewer_->addCoordinateSystem(1.0);
      // Register keyboard callback
      viewer_->registerKeyboardCallback(
          [&](const pcl::visualization::KeyboardEvent &event) -> void {
            KeyboardEventCallback(event);
          });
      Run();
    }));
  }

  virtual ~LidarVisualizer() {
    is_stopped_ = true;
    viewer_->close();
    if (thread_->joinable()) {
      thread_->join();
    }
  }

  void Render(const std::shared_ptr<LidarVisFrame> &frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    while (history_frames_.size() > max_history_size_) {
      history_frames_.pop_back();
    }
    history_frames_.emplace_front(frame);
    curr_frame_iter_ = history_frames_.begin();
    is_updated_ = true;
  }

  std::string name() const {
    return name_;
  }

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
   * @brief Draw objects. This method should be overrided for customization
   */
  virtual void Draw() {
    auto frame = GetCurrentFrame<LidarVisFrame>();
    DrawPointCloud<Point>(frame.cloud, WHITE, "point cloud");
  }

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

  void RemoveRenderedObjects() {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
  }

  template <typename FrameT>
  FrameT GetCurrentFrame() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return *static_cast<FrameT *>((*curr_frame_iter_).get());
  }

  template <typename PT>
  void DrawPointCloud(const typename pcl::PointCloud<PT>::ConstPtr &cloud,
                      const Color &color, const std::string &id,
                      int point_size = 3) {
    AddPointCloud<PT>(cloud, color, id, viewer_.get(), point_size);
  }

  template <typename PT>
  void DrawPointCloud(const typename pcl::PointCloud<PT>::ConstPtr &cloud,
                      const std::string &field, const std::string &id,
                      int point_size = 3) {
    AddPointCloud<PT>(cloud, field, id, viewer_.get(), point_size);
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
  std::deque<std::shared_ptr<LidarVisFrame>> history_frames_;

  // The current displayed frame iter.
  typename std::deque<std::shared_ptr<LidarVisFrame>>::iterator
      curr_frame_iter_;

  // thread for visualization
  std::unique_ptr<std::thread> thread_ = nullptr;

  // viewer
  std::unique_ptr<PCLVisualizer> viewer_ = nullptr;

  DISALLOW_COPY_AND_ASSIGN(LidarVisualizer);
};

}  // namespace common