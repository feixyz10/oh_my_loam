#pragma once

#include <vector>

#include "common/macro/macros.h"
#include "oh_my_loam/base/types.h"

namespace oh_my_loam {

class Row;

class Grid;

struct Index {
  int k, j, i;
};

class Map {
 public:
  Map(int z_size, int y_size, int x_size, double z_step, double y_step,
      double x_step);

  Grid &at(int z_ix);

  const Grid &at(int z_ix) const;

  Row &at(int z_ix, int y_ix);

  const Row &at(int z_ix, int y_ix) const;

  TPointCloudPtr &at(int z_ix, int y_ix, int x_ix);

  const TPointCloudPtr &at(int z_ix, int y_ix, int x_ix) const;

  TPointCloudPtr &at(const Index &index);

  const TPointCloudPtr &at(const Index &index) const;

  void Clear();

  void ShiftZ(int n);

  void ShiftY(int n);

  void ShiftX(int n);

  Index GetIndex(const TPoint &point);

  std::vector<Index> GetSurrIndices(const TPoint &point, int n);

  TPointCloudPtr GetSurrPoints(const TPoint &point, int n);

  TPointCloudPtr GetAllPoints() const;

 private:
  int z_size_, y_size_, x_size_;

  double z_step_, y_step_, x_step_;

  int z_center_, y_center_, x_center_;

  std::vector<Grid> map_;

  DISALLOW_COPY_AND_ASSIGN(Map);
};

class Row {
 public:
  explicit Row(int n);

  TPointCloudPtr &at(int ix);

  const TPointCloudPtr &at(int ix) const;

  void Clear();

  TPointCloudPtr GetAllPoints() const;

 private:
  std::vector<TPointCloudPtr> row_;
};

class Grid {
 public:
  Grid(int m, int n);

  void Clear();

  Row &at(int ix);

  const Row &at(int ix) const;

  TPointCloudPtr GetAllPoints() const;

 private:
  std::vector<Row> grid_;
};

}  // namespace oh_my_loam