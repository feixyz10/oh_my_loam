cmake_minimum_required(VERSION 3.10)
project(oh_my_loam)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Ceres REQUIRED)
find_package(PCL QUIET)
find_package(g3log REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${G3LOG_INCLUDE_DIRS}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(common)
add_subdirectory(oh_my_loam)
add_subdirectory(examples)
