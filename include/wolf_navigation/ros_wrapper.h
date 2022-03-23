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

#ifndef ROSWRAPPER_H
#define ROSWRAPPER_H

#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>

#include <wolf_navigation/estimators.h>

namespace wolf_navigation {

class RosWrapper
{

public:

  const std::string CLASS_NAME = "RosWrapper";

  /**
   * @brief Shared pointer to RosWrapper
   */
  typedef std::shared_ptr<RosWrapper> Ptr;

  /**
   * @brief Shared pointer to const RosWrapper
   */
  typedef std::shared_ptr<const RosWrapper> ConstPtr;


  RosWrapper(ros::NodeHandle& nh);

  ~RosWrapper() {}

  void init(const std::string& trackingcamera_frame_id, const std::string& child_frame_id, bool twist_in_local_frame = false);

  void callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

protected:

  std::string child_frame_id_;
  std::string odom_frame_id_;

  std::string trackingcamera_topic_;
  std::string odom_topic_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  geometry_msgs::TransformStamped transform_msg_out_;
  nav_msgs::Odometry odom_msg_out_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_publisher_;
  ros::Time t_;
  ros::Time t_prev_;

  TrackingCameraEstimator::Ptr single_tc_;

  Eigen::Isometry3d tmp_isometry3d_;
  Eigen::Vector3d tmp_vector3d_;
  Eigen::Vector3d tmp_vector3d_1_;

};

} // namespace

#endif
