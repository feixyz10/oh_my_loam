#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace oh_my_loam {

class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start_ = std::chrono::system_clock::now(); }

  double toc() {
    end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_ - start_;
    return elapsed_seconds.count() * 1000;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
};

}  // namespace oh_my_loam