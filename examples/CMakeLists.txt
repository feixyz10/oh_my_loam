find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
)

include_directories(SYSTEM
	${catkin_INCLUDE_DIRS} 
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(main_rosbag main_rosbag.cc)
target_link_libraries(main_rosbag
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${G3LOG_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  common
  oh_my_loam
  extractor
  odometer
  mapper
  solver
  ${CERES_LIBRARIES}
  visualizer
  base
)

add_executable(main_noros main_noros.cc)
target_link_libraries(main_noros
  ${PCL_LIBRARIES}
  ${G3LOG_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  common
  oh_my_loam
  extractor
  odometer
  mapper
  solver
  ${CERES_LIBRARIES}
  visualizer
  base
)