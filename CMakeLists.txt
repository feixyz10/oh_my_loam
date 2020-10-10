cmake_minimum_required(VERSION 2.8.3)
project(oh_loam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL QUIET)
if (NOT PCL_FOUND) 
  message(FATAL_ERROR "PCL not found.")
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/common
  ${PROJECT_SOURCE_DIR}/src
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE_DIR}/src/base_feature_extractor.cc
)