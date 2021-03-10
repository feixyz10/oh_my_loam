#pragma once

#include <iomanip>
#include <memory>
#include <mutex>

// format timestamp
#define FMT_TIMESTAMP(timestamp) std::fixed << std::setprecision(3) << timestamp

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

// adapted form baidu apollo cyber/common/macros.h
#define DECLARE_SINGLETON(classname) \
 public:                             \
  static classname *Instance() {     \
    static classname instance;       \
    return &instance;                \
  }                                  \
                                     \
 private:                            \
  classname();                       \
  DISALLOW_COPY_AND_ASSIGN(classname)
