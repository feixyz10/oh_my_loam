#pragma once

#include "common/pcl/pcl_types.h"

namespace common {
/**
 * @return Coefficients representing line equation ax + by + c = 0 (a^2 +
 * b^2 = 1)
 */
template <typename PT>
Eigen::Vector3d FitLine2D(const pcl::PointCloud<PT> &cloud,
                          double *const score = nullptr) {
  Eigen::MatrixX2f data(cloud.size(), 2);
  size_t i = 0;
  for (const auto &p : cloud) data.row(i++) << p.x, p.y;
  Eigen::RowVector2f centroid = data.colwise().mean();
  Eigen::MatrixX2f data_centered = data.rowwise() - centroid;
  Eigen::JacobiSVD<Eigen::MatrixX2f> svd(data_centered, Eigen::ComputeThinV);
  Eigen::Vector2f normal = svd.matrixV().col(1);
  float c = -centroid * normal;
  if (score) *score = svd.singularValues()[0] / svd.singularValues()[1];
  return {normal.x(), normal.y(), c};
}

/**
 * @return Coefficients with its first three components representing point
 * cloud's centroid and last three -- line direction
 */
template <typename PT>
Eigen::Matrix<double, 6, 1> FitLine3D(const pcl::PointCloud<PT> &cloud,
                                      double *const score = nullptr) {
  Eigen::MatrixX3f data(cloud.size(), 3);
  size_t i = 0;
  for (const auto &p : cloud) data.row(i++) << p.x, p.y, p.z;
  Eigen::RowVector3f centroid = data.colwise().mean();
  Eigen::MatrixX3f data_centered = data.rowwise() - centroid;
  Eigen::JacobiSVD<Eigen::MatrixX3f> svd(data_centered, Eigen::ComputeThinV);
  Eigen::Vector3f direction = svd.matrixV().col(0);
  Eigen::Matrix<double, 6, 1> line_coeffs;
  line_coeffs.topRows(3) = centroid.transpose().cast<double>();
  line_coeffs.bottomRows(3) = direction.cast<double>();
  if (score) *score = svd.singularValues()[0] / svd.singularValues()[1];
  return line_coeffs;
}

/**
 * @return Coefficients representing plane equation ax + by + cz + d = 0 (a^2 +
 * b^2 + c^2 = 1)
 */
template <typename PT>
Eigen::Vector4d FitPlane(const pcl::PointCloud<PT> &cloud,
                         double *const score = nullptr) {
  Eigen::MatrixX3f data(cloud.size(), 3);
  size_t i = 0;
  for (const auto &p : cloud) data.row(i++) << p.x, p.y, p.z;
  Eigen::RowVector3f centroid = data.colwise().mean();
  Eigen::MatrixX3f data_centered = data.rowwise() - centroid;
  Eigen::JacobiSVD<Eigen::MatrixX3f> svd(data_centered, Eigen::ComputeThinV);
  Eigen::Vector3f normal = svd.matrixV().col(2);
  float d = -centroid * normal;
  if (score) {
    *score = svd.singularValues()[1] * svd.singularValues()[1] /
             (svd.singularValues()[0] * svd.singularValues()[2]);
  }
  return {normal.x(), normal.y(), normal.z(), d};
}

}  // namespace common