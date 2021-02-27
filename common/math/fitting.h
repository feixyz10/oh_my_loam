#pragma once

#include <eigen3/Eigen/Dense>
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
  Eigen::Matrix2f cov_mat = data_centered.transpose() * data_centered;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov_mat);
  Eigen::Vector2f normal = solver.eigenvectors().col(0);
  double c = -centroid * normal;
  Eigen::Vector3d coeffs(normal(0), normal(1), c);
  if (score) {
    *score = solver.eigenvalues()[1] / (solver.eigenvalues()[0] + 1e-7);
  }
  return coeffs;
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
  Eigen::Matrix3f cov_mat = data_centered.transpose() * data_centered;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov_mat);
  Eigen::Vector3f direction = solver.eigenvectors().col(2);
  Eigen::Matrix<double, 6, 1> coeffs;
  coeffs.topRows(3) = centroid.transpose().cast<double>();
  coeffs.bottomRows(3) = direction.cast<double>();
  if (score) {
    *score = solver.eigenvalues()[2] / (solver.eigenvalues()[1] + 1e-7);
  }
  return coeffs;
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
  Eigen::Matrix3f cov_mat = data_centered.transpose() * data_centered;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov_mat);
  Eigen::Vector3f normal = solver.eigenvectors().col(0);
  double d = -centroid * normal;
  Eigen::Vector4d coeffs(normal(0), normal(1), normal(2), d);
  if (score)
    *score = solver.eigenvalues()[1] * solver.eigenvalues()[1] /
             (solver.eigenvalues()[2] * solver.eigenvalues()[0] + 1e-7);
  return coeffs;
}

}  // namespace common