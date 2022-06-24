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

#ifndef WOLF_NAVIGATION_UTILS_ODOM_PUBLISHER_H
#define WOLF_NAVIGATION_UTILS_ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "wolf_navigation_utils/estimators.h"

namespace wolf_navigation {

class OdomPublisher
{

public:

  const std::string CLASS_NAME = "OdomPublisher";

  /**
   * @brief Shared pointer to OdomPublisher
   */
  typedef std::shared_ptr<OdomPublisher> Ptr;

  /**
   * @brief Shared pointer to const OdomPublisher
   */
  typedef std::shared_ptr<const OdomPublisher> ConstPtr;


  OdomPublisher(ros::NodeHandle& nh);

  ~OdomPublisher() {}

  void init();

  void singleCameraCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

  void multiCameraCallback(const nav_msgs::Odometry::ConstPtr &odom_msg_1, const nav_msgs::Odometry::ConstPtr& odom_msg_2);

protected:

  std::string basefoot_frame_id_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string odom_topic_;
  bool basefoot_estimation_on_;

  ros::NodeHandle nh_;
  ros::Subscriber single_camera_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> multi_camera_0_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> multi_camera_1_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry,nav_msgs::Odometry>> multi_camera_sync_;
  nav_msgs::Odometry odom_msg_out_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_publisher_;
  ros::Time odom_pub_t_;
  ros::Time odom_pub_t_prev_;
  ros::Time basefoot_pub_t_;
  ros::Time basefoot_pub_t_prev_;
  std::vector<std::string> contact_names_;
  double initial_height_;

  std::vector<bool> contacts_;
  std::vector<double> heights_;
  Eigen::Isometry3d odom_T_base_;
  Eigen::Isometry3d odom_T_basefoot_;
  Eigen::Isometry3d basefoot_T_base_;

  std::vector<TrackingCameraEstimator::Ptr> camera_estimators_;
  BasefootEstimator basefoot_estimator_;

  Eigen::Isometry3d tmp_isometry3d_;
  Eigen::Vector3d tmp_vector3d_;
  Eigen::Vector3d tmp_vector3d_1_;
  Eigen::Vector6d tmp_vector6d_;
  Eigen::Matrix6d tmp_matrix6d_;
  Eigen::Matrix6d tmp_matrix6d_1_;

private:

  void updateBasefoot(const Eigen::Isometry3d& odom_T_base);

  void updateCamera(const unsigned int& camera_id, const nav_msgs::Odometry::ConstPtr& odom_msg);

  void publishOdom(const Eigen::Isometry3d &pose, const Eigen::Matrix6d &pose_cov, const Eigen::Vector6d twist, const Eigen::Matrix6d &twist_cov);

  void publishBasefoot(const Eigen::Isometry3d &pose);

};

} // namespace

#endif
