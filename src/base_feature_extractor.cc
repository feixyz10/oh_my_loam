#include "src/base_feature_extractor.h"

#include <cmath>
#include "common/filter.h"

namespace oh_loam {

const double kPointMinDist = 0.1;
const int kScanSegNum = 6;

bool FeaturePointsExtractor::Extract(const PointCloud& cloud_in,
                                     FeaturePoints* const feature) const {
  PointCloudPtr cloud(new PointCloud);
  RemoveNaNPoint<Point>(cloud_in, cloud.get());
  RemoveClosedPoints<Point>(*cloud, cloud.get(), kPointMinDist);
  std::vector<IPointCloud> scans;
  ScanSplit(*cloud, &scans);
  for (auto& scan : sccans) {
    ComputeCurvature(&scan);
    AssignType(&scan);
  }
  for (const auto& scan : sccans) {
    *(feature->laser_cloud) += scan;
    for (const auto& pt : scan.points) {
      switch (pt.type) {
        case PointType::FLAT:
          feature->flat_surf_points.emplace_back(pt);
          break;
        case PointType::LESS_FLAT:
          feature->less_flat_surf_points.emplace_back(pt);
          break;
        case PointType::LESS_SHARP:
          feature->less_sharp_corner_points.emplace_back(pt);
          break;
        case PointType::SHARP:
          feature->sharp_corner_points.emplace_back(pt);
          break;
        default:
          break;
      }
    }
  }
}

void FeaturePointsExtractor::SplitScan(
    const PointCloud& cloud, std::vector<IPointCloud>* const scans) const {
  scans.resize(num_scans_);
  const auto & [ yaw_start, yaw_end ] = GetYawRange(cloud);
  const double yaw_range = yaw_end - yaw_start;
  bool half_passed = false;
  for (const auto& pt : cloud.points) {
    int scan_id = GetScanID(pt);
    if (scan_id >= num_scans_ || scan_id < 0) continue;
    double yaw = -atan2(pt.y, pt.x);
    double yaw_diff = NormalizeAngle(yaw - yaw_start);
    if (yaw_diff > 0) {
      if (half_passed) yaw_start += 2 * M_PI;
    } else {
      half_passed = true;
      yaw_start += 2 * M_PI;
    }
    scans[scan_id].emplace_back(pt.x, pt.y, pt.z, yaw_diff / yaw_range);
  }
}

void FeaturePointsExtractor::ComputePointCurvature(
    IPointCloud* const scan) const {
  auto& pts = scan->points;
  for (int i = 5; i < pts.size() - 5; ++i) {
    float diffX = pts[i - 5].x + pts[i - 4].x + pts[i - 3].x + pts[i - 2].x +
                  pts[i - 1].x + pts[i + 1].x + pts[i + 2].x + pts[i + 3].x +
                  pts[i + 4].x + pts[i + 5].x - 10 * pts[i].x;
    float diffY = pts[i - 5].y + pts[i - 4].y + pts[i - 3].y + pts[i - 2].y +
                  pts[i - 1].y + pts[i + 1].y + pts[i + 2].y + pts[i + 3].y +
                  pts[i + 4].y + pts[i + 5].y - 10 * pts[i].y;
    float diffZ = pts[i - 5].z + pts[i - 4].z + pts[i - 3].z + pts[i - 2].z +
                  pts[i - 1].z + pts[i + 1].z + pts[i + 2].z + pts[i + 3].z +
                  pts[i + 4].z + pts[i + 5].z - 10 * pts[i].z;
    pts[i].curvature = diffX * diffX + diffY * diffY + diffZ * diffZ;
  }
}

void FeaturePointsExtractor::AssignPointType(IPointCloud* const scan) const {
  int pt_num = scan->size();
  int pt_num_seg = pt_num / kScanSegNum;
  std::vector<bool> picked(pt_num, false);
  for (int i = 0; i < kScanSegNum; ++i) {
    int begin = i * pt_num_seg;
    int end = i * pt_num_seg + pt_num_seg;
  }
}

}  // oh_loam
