cmake_minimum_required(VERSION 3.10)
project(oh_my_loam)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -lpthread")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Ceres REQUIRED)
find_package(PCL QUIET)
find_package(g3log REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   roscpp
#   rospy
#   rosbag
#   std_msgs
#   image_transport
#   cv_bridge
#   tf
# )

include_directories(SYSTEM
	# ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${G3LOG_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# catkin_package(
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#   DEPENDS EIGEN3 PCL 
#   INCLUDE_DIRS src common
# )

include_directories(
  ${CMAKE_SOURCE_DIR}
)

add_subdirectory(common)
add_subdirectory(oh_my_loam)

# add_executable(main main.cc)
# target_link_libraries(main
#   ${catkin_LIBRARIES} 
#   ${PCL_LIBRARIES}
#   ${G3LOG_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   common
#   oh_my_loam
#   extractor
#   odometry
#   mapper
#   solver
#   ${CERES_LIBRARIES}
#   helper
#   visualizer
# )
