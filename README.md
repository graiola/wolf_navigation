# WoLF: Whole-body Locomotion Framework for quadruped robots

This package contains the navigation stack to be used with WoLF.

## Mantainers:

Federico Rollo, Gennaro Raiola

## Set velodyne gpus dependencies for gazebo

To use the velodyne simulation on gpu on gazebo type the following command in the terminal: 
 ```
 sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
 sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
 sudo apt update
 sudo apt upgrade
 ```
## Dependencies:

- pointcloud-to-laserscan
- velodyne-laserscan
- velodyne-pointcloud
- velodyne-gazebo-plugins
- velodyne-description
- hector-mapping
- move-base
- explore-lite
- dwa-local-planner
- amcl
- map-server
