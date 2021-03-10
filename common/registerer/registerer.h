#pragma once

#include <map>
#include <memory>
#include <string>

#include "common/log/log.h"

namespace common {

template <typename BaseClass>
class AbstractFactory {
 public:
  virtual ~AbstractFactory() = default;
  virtual BaseClass *Create() = 0;
};

template <typename BaseClass, typename DerivedClass>
class ConcreteFactory : public AbstractFactory<BaseClass> {
 public:
  BaseClass *Create() override {
    return new DerivedClass;
  }
};

template <typename BaseClass>
inline auto &GetFactoryMap() {
  static std::map<std::string, std::shared_ptr<AbstractFactory<BaseClass>>> map;
  return map;
}

template <typename BaseClass>
class Registerer {
 public:
  template <typename DerivedClass>
  static void Register(const std::string &derived_class_name) {
    static_assert(std::is_base_of<BaseClass, DerivedClass>::value, "");
    auto &factory_map = GetFactoryMap<BaseClass>();
    if (factory_map.find(derived_class_name) != factory_map.end()) {
      return;
    }
    factory_map[derived_class_name].reset(
        new ConcreteFactory<BaseClass, DerivedClass>);
  }

  static BaseClass *NewInstance(const std::string &derived_class_name) {
    auto &factory_map = GetFactoryMap<BaseClass>();
    auto iter = factory_map.find(derived_class_name);
    if (iter == factory_map.end()) {
      AFATAL << "Class not registered: " << derived_class_name << ".";
      return nullptr;
    }
    return iter->second->Create();
  }

  static bool IsRegistered(const std::string &derived_class_name) {
    auto &factory_map = GetFactoryMap<BaseClass>();
    return factory_map.find(derived_class_name) != factory_map.end();
  }
};

}  // namespace common

#define REGISTER_CLASS(base_class, derived_class)                              \
  namespace {                                                                  \
  __attribute__((constructor)) void Register##derived_class() {                \
    ::common::Registerer<base_class>::Register<derived_class>(#derived_class); \
  }                                                                            \
  }  // namespace
