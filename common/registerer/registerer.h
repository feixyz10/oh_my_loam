#pragma once

#include <map>
#include <memory>
#include <string>

#include "common/log/log.h"
#include "common/macro/macros.h"

template <typename BaseClass>
class AbstractFactory {
  virtual ~AbstractFactory() = default;
  virtual std::shared_ptr<BaseClass> Create() = 0;
};

template <typename BaseClass, typename DerivedClass>
class ConcreteFactory {
  virtual std::shared_ptr<BaseClass> Create() = 0;
};