# Oh-My-LOAM

Oh-My-LOAM is a ROS-free implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time). 
This implementation is modified from [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM). 

Comparing with A-LOAM, this implementation has following features:

- ROS-free: it can run without ROS environment
- Multi-threading instead of multi-process: more deterministic
- Higher code quality: more readable and easier to understand/modify

<img src="images/nsh_indoor_outdoor.png" alt="nsh_indoor_outdoor" height="500" align="bottom" />

# How to run
## BUILD

Install dependences (listed below).\
Clone this repository\
Compile: 
```bash
mkdir build && cd build
cmake ..
make -j6
```

## Run with ROS bag as input
Although **Oh-My-LOAM** is ROS-free, running it with ROS bag as input is the simplest way.
We'll take *nsh_indoor_outdoor.bag* as example. 
You can download this bag from [google drive](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) or [baidupan](https://pan.baidu.com/s/1TbfMQ3Rvmmn1hCjFhXSPcQ) (提取码:9nf7).

Launch **Oh-My-LOAM**: 
```
./devel/lib/oh_my_loam/main_rosbag ../configs/config_nsh_indoor_outdoor.yaml
```
Play ROS bag (in a new terminal):
```
ros play nsh_indoor_outdoor.bag
```

## Run without ROS support
Launch **Oh-My-LOAM**:
```
./devel/lib/oh_my_loam/main_noros ../configs/config_nsh_indoor_outdoor.yaml xxxxxx
```
Please replace `xxxxxx` with the directory that contains the input point cloud files with tree structure like following: 
```
xxxxxx
├── frame00000.pcd               
├── frame00001.pcd               
├── frame00002.pcd               
├── frame00003.pcd               
├── frame00004.pcd
├── ...          
```
Currently only `.pcd` format is supported. 
You can modify `examples/main_noros.cc` to add support for other point cloud formats.  

# Dependences

### OS
Tested on ubuntu 16.04/18.04/20.04.

### C++17
If cannot find *std::filesystem* error is encountered during your compiling, please upgrade your compiler. 
We recommend `g++-9` (or higher version).

### ROS
Only `examples/main_rosbag.cc` needs ROS. You can exclude it from compiling by modifying `examples/CMakeLists.txt`.

### Eigen: linear algebra, quaternion
```
sudo apt install libeigen3-dev
```

### pcl: point cloud processing
```
sudo apt install libpcl-dev
```

### g3log: logging
Follow [g3log](https://github.com/KjellKod/g3log) to install.

### yaml-cpp: yaml parsing
```
sudo apt install libyaml-cpp-dev
```

### ceres: non-linear optimization
```
sudo apt install libceres-dev
```