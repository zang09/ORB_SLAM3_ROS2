# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

---

## Demo Video
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## Prerequisites
- I have tested on below version.
  - Ubuntu 20.04
  - ROS2 foxy
  - OpenCV 4.2.0

- Build ORB_SLAM3
  - Go to this [repo](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED) and follow build instruction.

- Install related ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## How to build
1. Clone repository to your ROS workspace
```
$ mkdir -p colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

2. Change this [line](https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeLists.txt#L5) to your own `python site-packages` path

3. Change this [line](https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeModules/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

Now, you are ready to build!
```
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select orbslam3
```

## Troubleshootings
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```

2. Please compile with `OpenCV 4.2.0` version.
Refer this [#issue](https://github.com/zang09/ORB_SLAM3_ROS2/issues/2#issuecomment-1251850857)

## How to use
1. Source the workspace  
```
$ source ~/colcon_ws/install/local_setup.bash
```

2. Run orbslam mode, which you want.  
This repository only support `MONO, STEREO, RGBD, STEREO-INERTIAL` mode now.  
You can find vocabulary file and config file in here. (e.g. `orbslam3_ros2/vocabulary/ORBvoc.txt`, `orbslam3_ros2/config/monocular/TUM1.yaml` for monocular SLAM).
  - `MONO` mode  
```
$ ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO` mode  
```
$ ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```
  - `RGBD` mode  
```
$ ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO-INERTIAL` mode  
```
$ ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

## Run with rosbag
To play ros1 bag file, you should install `ros1 noetic` & `ros1 bridge`.  
Here is a [link](https://www.theconstructsim.com/ros2-qa-217-how-to-mix-ros1-and-ros2-packages/) to demonstrate example of `ros1-ros2 bridge` procedure.  
If you have `ros1 noetic` and `ros1 bridge` already, open your terminal and follow this:  
(Shell A, B, C, D is all different terminal, e.g. `stereo-inertial` mode)
1. Download EuRoC Dataset (`V1_02_medium.bag`)
```
$ wget -P ~/Downloads http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
```  

2. Launch Terminal  
(e.g. `ROS1_INSTALL_PATH`=`/opt/ros/noetic`, `ROS2_INSTALL_PATH`=`/opt/ros/foxy`)
```
#Shell A:
source ${ROS1_INSTALL_PATH}/setup.bash
roscore

#Shell B:
source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge

#Shell C:
source ${ROS1_INSTALL_PATH}/setup.bash
rosbag play ~/Downloads/V1_02_medium.bag --pause /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu

#Shell D:
source ${ROS2_INSTALL_PATH}/setup.bash
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

3. Press `spacebar` in `Shell C` to resume bag file.  

## Acknowledgments
This repository is modified from [this](https://github.com/curryc/ros2_orbslam3) repository.  
To add `stereo-inertial` mode and improve build difficulites.
