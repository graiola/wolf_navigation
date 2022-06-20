# WoLF: Whole-body Locomotion Framework for quadruped robots

This package contains the navigation stack to be used with [WoLF](https://github.com/graiola/wolf-setup).

## Mantainers:

Federico Rollo, Gennaro Raiola

## How to use it:
The package provides the following features:
1) Navigation in known and unknown enviroments
2) Waypoints selection for navigation
3) Exploration and mapping

#### 1) Navigation:

![spot_nav](https://user-images.githubusercontent.com/76060218/153372357-cba270e2-ee80-4032-a45b-91c43fe6bcfb.png)

To perform navigation in a known enviroment you need a map already created and saved in `https://github.com/graiola/wolf_navigation/tree/master/wolf_navigation_utils/maps`.

To launch the navigation in the saved map type in the terminal:

```
roslaunch wolf_navigation_utils wolf_navigation.launch map_file:=MAP_FILE
```

otherwise, to perform navigation in a unknown enviroment: 

```
roslaunch wolf_navigation_utils wolf_navigation.launch mapping:=true
```

#### 2) Waypoints:


#### 3) Exploration:


#### Notes:

To select the gazebo environment and the robot to use please take a look at the [wolf_gazebo_resources](https://github.com/graiola/wolf_gazebo_resources) and
[wolf_descriptions](https://github.com/graiola/wolf_descriptions) packages.

## Dependencies:

#### Set velodyne gpus dependencies for gazebo:

To enable the velodyne simulation using the gpu with gazebo type the following commands in the terminal: 
 ```
 sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
 sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
 ```
 ```
 sudo apt update &&  sudo apt upgrade -y
 ```

#### ROS dependencies:

To install the required ros dependencies use the following command:

```
sudo apt-get install ros-${ROS_DISTRO}-pointcloud-to-laserscan \
                     ros-${ROS_DISTRO}-velodyne-laserscan      \
                     ros-${ROS_DISTRO}-velodyne-pointcloud     \
                     ros-${ROS_DISTRO}-velodyne-gazebo-plugins \
                     ros-${ROS_DISTRO}-velodyne-description    \
                     ros-${ROS_DISTRO}-hector-mapping          \
                     ros-${ROS_DISTRO}-move-base               \
                     ros-${ROS_DISTRO}-dwa-local-planner       \
                     ros-${ROS_DISTRO}-teb-local-planner       \
                     ros-${ROS_DISTRO}-map-server              \
                     ros-${ROS_DISTRO}-amcl                    \
                     ros-${ROS_DISTRO}-gmapping                \
                     ros-${ROS_DISTRO}-costmap-converter       \
                     ros-${ROS_DISTRO}-costmap-2d              \
                     -y
```
