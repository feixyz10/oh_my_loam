#pragma once

#include <cmath>
#include <vector>

namespace common {

// normalize an angle to [-pi, pi)
double NormalizeAngle(double ang);

// Convert an angle from degree to rad
double Degree2Rad(double degree);

// Convert an angle from rad to degree
double Rad2Degree(double rad);

// like Python built-in range, [begin, end)
const std::vector<int> Range(int begin, int end, int step = 1);

// like Python built-in range, [0, end)
const std::vector<int> Range(int end);

}  // namespace common