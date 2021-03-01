# Oh-My-LOAM

Oh-My-LOAM is a ROS-free implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time). This implementation is modified from [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM). 

Comparing with A-LOAM, this implementation has following features:

- it's ROS-free
- it's more readable and easier to understand/modify

# Dependences

- Eigen: linear algebra, quaternion
- pcl: point cloud processing
- g3log: logging
- yaml-cpp: yaml parsing
- ceres: non-linear optimization
- c++17

