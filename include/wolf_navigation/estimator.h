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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>

namespace wolf_navigation
{

class EstimatorInterface
{

public:

  const std::string CLASS_NAME = "EstimatorInterface";

  /**
   * @brief Shared pointer to Estimator
   */
  typedef std::shared_ptr<EstimatorInterface> Ptr;

  /**
   * @brief Shared pointer to const Estimator
   */
  typedef std::shared_ptr<const EstimatorInterface> ConstPtr;


  EstimatorInterface(ros::NodeHandle& n, const std::string& odom_topic = "/odom");

  virtual ~EstimatorInterface() {}

  virtual void update() = 0;

protected:

  geometry_msgs::TransformStamped transform_msg_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_publisher_;
  ros::Time t_;
  ros::Time t_prev_;

};

class TrackingCamera : public EstimatorInterface
{

  const std::string CLASS_NAME = "TrackingCamera";

  /**
   * @brief Shared pointer to TrackingCamera
   */
  typedef std::shared_ptr<TrackingCamera> Ptr;

  /**
   * @brief Shared pointer to const TrackingCamera
   */
  typedef std::shared_ptr<const TrackingCamera> ConstPtr;

  TrackingCamera();

  virtual ~TrackingCamera() {}

  virtual void update(const nav_msgs::Odometry_::ConstPtr &trackingcamera_odom_msg);

private:

  Eigen::Isometry3d trackingcamera_T_basefootprint_; // transform
  Eigen::Isometry3d odom_T_trackingcamera_; // transform
  Eigen::Isometry3d odom_T_basefootprint_; // transform
  Eigen::Vector3d trackingcamera_v_; // twist linear
  Eigen::Vector3d trackingcamera_omega_; // twist angular
  Eigen::Vector3d basefootprint_v_; // twist linear
  Eigen::Vector3d basefootprint_omega_; // twist angular
  Eigen::Matrix3d basefootprint_R_odom_; // rotation matrix
  Eigen::Matrix3d basefootprint_R_trackingcamera_; // rotation matrix

};


} // namespace

#endif
