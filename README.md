# WoLF: Whole-body Locomotion Framework for quadruped robots

This package contains the navigation stack to be used with [WoLF](https://github.com/graiola/wolf-setup).

## Mantainers:

Federico Rollo, Gennaro Raiola

## How to use it
The package is composed by three main feature:
1) Navigation through a know map
2) Mapping moving the robot with external commands (Keyboard, twist, ...)
3) Exploration (Mapping with autonomous searching for new spaces)

#### 1) Navigation

![spot_nav](https://user-images.githubusercontent.com/76060218/153372357-cba270e2-ee80-4032-a45b-91c43fe6bcfb.png)

To perform navigation you need a map already done and saved. In order to use it you'll need to load the gazebo 
environment using the ```world_name:=``` argument otherwise an empty world will be loaded

TODO: load an empty map if no world has been provided \
TODO: explain where to save maps to be loaded (the map should have the same name of the world if you use gazebo)

To launch the navigation mode type in the terminal:
```
roslaunch wolf_navigation wolf_navigation world_name:=YOUR_WORLD_NAME
```
#### 2) Mapping

![spot_map](https://user-images.githubusercontent.com/76060218/153372856-dcd3450b-5202-4e4e-8b88-dd97eb1b3142.png)

To perform mapping you need at least a gazebo world to be loaded, and you have to load it with the ```world_name:=``` 
argument. 

To launch the mapping mode type in the terminal:
```
roslaunch wolf_navigation wolf_navigation world_name:=YOUR_WORLD_NAME mapping:=true
```

Once you have finished mapping your environment, or you are satisfied of your work, save your map in the ```maps```
folder using the same name of the world you have used (e.g., if I use office world, save the map as office).

```
rosrun map_server map_saver YOUR_WORLD_NAME.yaml
```

TODO: insert a reference to wolf_gazebo_resources?

#### 3) Exploration
This module is dependent on the mapping one, so you have the same necessity of that module.

To launch the exploration mode type in the terminal:
```
roslaunch wolf_navigation wolf_exploration world_name:=YOUR_WORLD_NAME
```

#### other information
To set the gazebo environment and the robot to use let's take a look at the ```wolf_gazebo_resources``` and the
```wolf_description``` packages

 
## Dependencies:

#### Set velodyne gpus dependencies for gazebo

To use the velodyne simulation on gpu on gazebo type the following command in the terminal: 
 ```
 sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
 sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
 sudo apt update
 sudo apt upgrade
 ```
#### ROS dependencies

```
sudo apt-get install ros-${ROS_DISTRO}-pointcloud-to-laserscan \
                     ros-${ROS_DISTRO}-velodyne-laserscan      \
                     ros-${ROS_DISTRO}-velodyne-pointcloud     \
                     ros-${ROS_DISTRO}-velodyne-gazebo-plugins \
                     ros-${ROS_DISTRO}-velodyne-description    \
                     ros-${ROS_DISTRO}-hector-mapping          \
                     ros-${ROS_DISTRO}-move-base               \
                     ros-${ROS_DISTRO}-explore-lite            \
                     ros-${ROS_DISTRO}-dwa-local-planner       \
                     -y
```
