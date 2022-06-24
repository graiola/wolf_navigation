/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola, Federico Rollo
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

/*
 * @brief odom_publisher_node.cpp
 * Info about the odometry for the ROS navigation stack can be found here: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 */

#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>

#include "wolf_navigation_utils/odom_publisher.h"

#define NODE_NAME "odom_publisher_node"

int main(int argc, char** argv)
{
  std::string ns = NODE_NAME;

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  wolf_navigation::OdomPublisher odom_publisher(n);
  odom_publisher.init();

  ros::spin();

  return 0;
}
