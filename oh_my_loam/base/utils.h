#pragma once

#include "common/geometry/pose3d.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

inline int GetScanId(const TPoint &pt) {
  return static_cast<int>(pt.time);
}

inline float GetTime(const TPoint &pt) {
  return pt.time - GetScanId(pt);
}

/**
 * @brief Transform a lidar point to the start of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 */
void TransformToStart(const common::Pose3d &pose, const TPoint &pt_in,
                      TPoint *const pt_out);

TPoint TransformToStart(const common::Pose3d &pose, const TPoint &pt_in);

/**
 * @brief Transform a lidar point to the end of the scan
 *
 * @param pose Relative pose, end scan time w.r.t. start scan time
 */
void TransformToEnd(const common::Pose3d &pose, const TPoint &pt_in,
                    TPoint *const pt_out);

TPoint TransformToEnd(const common::Pose3d &pose, const TPoint &pt_in);

}  // namespace oh_my_loam