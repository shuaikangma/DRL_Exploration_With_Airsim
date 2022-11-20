# DRL_Exploration_With_Airsim

### 本项目的仿真环境是基于AirSim进行开发的，其中视觉SLAM部分参考[ORB_SLAM_dense_map](),在这两个项目的基础上实现了深度强化学习的探索决策模块;最近忙于新的课题，对于项目内容疏于打理，暂时将完整项目上传上来，作为一个参考


# 测试命令
## 手柄控制
```bash
./Blocks.sh -ResX=1080 -ResY=720 -windowed
roslaunch airsim_ros_pkgs airsim_node.launch
roslaunch drone_exploration uav_joy.launch
python DRL_Exploration_With_Airsim/ros/src/drone_exploration/scripts/main_joy_control.py
roslaunch airsim_ros_pkgs rviz.launch
roslaunch drone_exploration octomap.launch

rqt
```

## 键盘控制
```bash
./Blocks.sh -ResX=1080 -ResY=720 -windowed
roslaunch airsim_ros_pkgs airsim_node.launch
roslaunch airsim_ros_pkgs rviz.launch
roslaunch drone_exploration octomap.launch
python DRL_Exploration_With_Airsim/ros/src/drone_exploration/scripts/main_keyboard_control.py

rqt
```

## 自主运行
```bash
./Blocks.sh -ResX=1080 -ResY=720 -windowed
roslaunch airsim_ros_pkgs airsim_node.launch
roslaunch airsim_ros_pkgs rviz.launch
roslaunch drone_exploration octomap.launch
python DRL_Exploration_With_Airsim/ros/src/drone_exploration/scripts/ppo_resnet3d_test_single.py

rqt
```

## 训练
```bash
./Blocks.sh -ResX=1080 -ResY=720 -windowed
roslaunch airsim_ros_pkgs airsim_node.launch
nohup python ppo_resnet3d_train.py > log/ppo_3d.log 2>&1 &

rqt
```

# 本项目的AirSim的配置和使用可以参考以下内容

[Home - AirSim](https://microsoft.github.io/AirSim/)

[https://github.com/microsoft/AirSim](https://github.com/microsoft/AirSim)

[https://github.com/shuaikangma/AirSim](https://github.com/shuaikangma/AirSim)

1. ****Download Binaries****

[Download Binaries - AirSim](https://microsoft.github.io/AirSim/use_precompiled/)

```cpp
./Building_99.sh -ResX=640 -ResY=480 -windowed
```

1. **Build for ROS**

[ROS: AirSim ROS Wrapper - AirSim](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)

```cpp
# make sure GCC version is 8 or above
sudo apt-get install gcc-8 g++-8
gcc-8 --version

#Install ROS melodic
#Install tf2 sensor and mavros packages:
sudo apt-get install ros-melodic-tf2-sensor-msgs ros-melodic-tf2-geometry-msgs ros-melodic-mavros*
#Install catkin_tools
sudo apt-get install python-catkin-tools #or pip install catkin_tools

#Build AirSim
#git clone https://github.com/Microsoft/AirSim.git
#cd AirSim
#error may occur because of network setting, use vpn
./setup.sh 
./build.sh

#Build ROS package
cd ros
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
#catkin build XXX_pkgs -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8

#Running and testing
source devel/setup.bash;
roslaunch airsim_ros_pkgs airsim_node.launch;
roslaunch airsim_ros_pkgs rviz.launch;
#Note: If you get an error running roslaunch airsim_ros_pkgs airsim_node.launch, run catkin clean and try again
```

1. **Runing ROS**

[ROS: AirSim Tutorial Packages - AirSim](https://microsoft.github.io/AirSim/airsim_tutorial_pkgs/)

1. **Control Move**
    1. call **/airsim_node/takeoff** service
    2. publish **/airsim_node/vel_cmd_body_frame** or **/airsim_node/vel_cmd_world_frame** to set velocity
    
    ```bash
    source PATH_TO/AirSim/ros/devel/setup.bash
    roscd airsim_tutorial_pkgs
    #cp settings/two_drones_camera_lidar_imu.json ~/Documents/AirSim/settings.json
    cp settings/single_drone_camera_lidar_imu.json ~/Documents/AirSim/settings.json
    cp settings/single_uav_depth.json ~/Documents/AirSim/settings.json
    cp settings/single_uav_lidar.json ~/Documents/AirSim/settings.json
    
    ## Start your unreal package or binary here
    cd ~/AirSim_Ur/Building99/LinuxNoEditor
    ./Building_99.sh -ResX=1080 -ResY=720 -windowed
    
    cd ~/AirSim_Ur/Wall_block/LinuxNoEditor
    ./Wall_Blocks.sh -ResX=1080 -ResY=720 -windowed
    
    cd ~/AirSim_Ur/Exploration/LinuxNoEditor
    ./Blocks.sh -ResX=720 -ResY=480 -windowed
    
    **roslaunch airsim_ros_pkgs airsim_node.launch**
    **roslaunch airsim_ros_pkgs rviz.launch**
    #roslaunch airsim_ros_pkgs uav_joy.launch
    roslaunch drone_exploration uav_joy.launch
    rqt
    ```
    
2. Reset Env
    
    [https://github.com/shuaikangma/AirSim/commit/88258528d094b2fdec1bc0f3a56d423bd12491e0](https://github.com/shuaikangma/AirSim/commit/88258528d094b2fdec1bc0f3a56d423bd12491e0)
    

3. Collision detect
    
    done
    
    [https://github.com/shuaikangma/AirSim/commit/88258528d094b2fdec1bc0f3a56d423bd12491e0](https://github.com/shuaikangma/AirSim/commit/88258528d094b2fdec1bc0f3a56d423bd12491e0)
    

4. Bug 
    
    If the drone go without control , reboot, may beacuse json
    
5. SLAM
    
    [https://github.com/microsoft/AirSim/issues/2369](https://github.com/microsoft/AirSim/issues/2369)
    
6. LogTemp: Error: Could not open output file to write voxel grid!
    
    版本问题，2020年之前版本可以的
    

7. 从深度图到点云的相机畸变问题

[Point cloud generation from depth images · Discussion #3955 · microsoft/AirSim](https://github.com/microsoft/AirSim/discussions/3955)

[https://github.com/microsoft/AirSim/issues/1198](https://github.com/microsoft/AirSim/issues/1198)

11.相机标定

[https://github.com/microsoft/AirSim/issues/269](https://github.com/microsoft/AirSim/issues/269)


# ORB_SLAM的配置参考一下内容

[ORB_SLAM3 + ROS采坑实录](https://blog.csdn.net/weixin_44128351/article/details/122721829)

## 遇到的小问题及解决办法

### 1  fatal error: pcl/point_types.h

[Unable to #include pcl files in header - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/123261/unable-to-include-pcl-files-in-header/)

```bash
find_package( PCL REQUIRED )

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)
```

### 2 install PCL-1.12

```bash
#download pcl-1.12 in https://github.com/PointCloudLibrary/pcl/releases
cd pcl-pcl-1.7.2 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j16
sudo make -j16 install

#if error: ‘Index’ is not a member of ‘Eigen’
install eigen3.3.x
```


## 测试

```bash
#ROB_SLAM3 ros
roscore
rqt
cd ORB_SLAM3_dense_map
rosbag play --loop ../rgbd_dataset_freiburg1_xyz.bag /camera/rgb/image_color:=/airsim_node/Drone_1/front_left_custom/Scene /camera/depth/image:=/airsim_node/Drone_1/front_left_custom/DepthPlanar
rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml

#ROB_SLAM3
cd ORB_SLAM3_dense_map
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ../rgbd_dataset_freiburg1_xyz ../rgbd_dataset_freiburg1_xyz/associations.txt
```


# DRL探索部分
    代码内容位于DRL_Exploration_With_Airsim/ros/src/drone_exploration