#pragma once

#include <sys/stat.h>

#include <mutex>
#include <shared_mutex>
#include <vector>

#include "common/geometry/pose3d.h"
#include "oh_my_loam/base/types.h"
#include "oh_my_loam/base/utils.h"
#include "oh_my_loam/mapper/map.h"
#include "oh_my_loam/solver/solver.h"

namespace oh_my_loam {

class Mapper {
 public:
  Mapper() = default;

  bool Init();

  void Process(double timestamp, const TPointCloudConstPtr &cloud_corn,
               const TPointCloudConstPtr &cloud_surf,
               const common::Pose3d &pose_curr2odom,
               common::Pose3d *const pose_curr2map);

  TPointCloudPtr GetMapCloudCorn() const;

  TPointCloudPtr GetMapCloudSurf() const;

  TPointCloudPtr GetMapCloud() const;

  void Reset();

 private:
  enum State { DONE, RUNNING, UN_INIT };

  void Run(const TPointCloudConstPtr &cloud_corn,
           const TPointCloudConstPtr &cloud_surf,
           const common::Pose3d &pose_init);

  void MatchCorn(const pcl::KdTreeFLANN<TPoint> &kdtree,
                 const TPointCloudConstPtr &cloud_curr,
                 const common::Pose3d &pose_curr2map,
                 std::vector<PointLinePair> *const pairs) const;

  void MatchSurf(const pcl::KdTreeFLANN<TPoint> &kdtree,
                 const TPointCloudConstPtr &cloud_curr,
                 const common::Pose3d &pose_curr2map,
                 std::vector<PointPlaneCoeffPair> *const pairs) const;

  State GetState() const;

  void SetState(State state);

  void AdjustMap(const TPoint &center);

  void UpdateMap(const common::Pose3d &pose_curr2map,
                 const TPointCloudConstPtr &cloud_corn = nullptr,
                 const TPointCloudConstPtr &cloud_surf = nullptr);

  void MatchCorn(const TPointCloudConstPtr &src,
                 const TCTPointCloudConstPtr &tgt,
                 std::vector<PointLinePair> *const pairs) const;

  void MatchSurf(const TPointCloudConstPtr &src,
                 const TCTPointCloudConstPtr &tgt,
                 std::vector<PointLinePair> *const pairs) const;
  // map
  mutable std::mutex map_mutex_;
  std::unique_ptr<Map> corn_map_;
  std::unique_ptr<Map> surf_map_;
  // state
  mutable std::mutex state_mutex_;
  State state_ = UN_INIT;
  common::Pose3d pose_odom2map_;
  // thread to run mapping
  mutable std::unique_ptr<std::thread> thread_{nullptr};
  // configs
  YAML::Node config_;
  std::vector<int> map_shape_, submap_shape_;
  double map_step_;
  bool is_vis_ = false;
  bool verbose_ = false;

  DISALLOW_COPY_AND_ASSIGN(Mapper)
};

}  // namespace oh_my_loam