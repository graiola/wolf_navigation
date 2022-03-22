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

#include <wolf_navigation/ros_wrapper.h>

int main(int argc, char** argv)
{
  std::string ns = "odometry_publisher";

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  // get ROS params
  std::vector<std::string> trackingcamera_topics;
  std::string basefootprint_frame_id;
  std::string odom_output_topic = "/odom";
  bool twist_in_local_frame;
  n.getParam("trackingcamera_topics", trackingcamera_topics);
  n.getParam("basefootprint_frame_id", basefootprint_frame_id);
  n.getParam("twist_in_local_frame", twist_in_local_frame);
  n.getParam("odom_output_topic",odom_output_topic);

  wolf_navigation::RosWrapper wrapper(n);
  wrapper.init(trackingcamera_topics[0],basefootprint_frame_id);

  ros::spin();

  return 0;
}
