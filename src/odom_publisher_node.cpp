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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


static std::string _trackingcamera_frame_id = "";
static std::string _base_footprint_frame_id = "";
static std::string _odom_frame_id = "";
static bool _twist_in_odom_frame = true; // By default, the libgazebo_ros_p3d plugin provides the twist wrt odom while move_base wants the twist defined wrt child_frame_id
boost::shared_ptr<OdomEstimatorType> sync_;


struct RosTransformHandler {

  typedef std::shared_ptr<RosTransformHandler> Ptr;

  RosTransformHandler(ros::NodeHandle& n, const std::string& odom_topic = "/odom"): tf_listener_(tf_buffer_) {odom_publisher_ = n.advertise<nav_msgs::Odometry>(odom_topic,20);}

  Eigen::Isometry3d trackingcamera_T_basefootprint_; // transform
  Eigen::Isometry3d odom_T_trackingcamera_; // transform
  Eigen::Isometry3d odom_T_basefootprint_; // transform
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::TransformStamped transform_msg_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_publisher_;
  Eigen::Vector3d trackingcamera_v_; // twist linear
  Eigen::Vector3d trackingcamera_omega_; // twist angular
  Eigen::Vector3d basefootprint_v_; // twist linear
  Eigen::Vector3d basefootprint_omega_; // twist angular
  Eigen::Matrix3d basefootprint_R_odom_; // rotation matrix
  Eigen::Matrix3d basefootprint_R_trackingcamera_; // rotation matrix
};

static RosTransformHandler::Ptr _handler;
static ros::Time _t;
static ros::Time _t_prev;

void callback(const nav_msgs::Odometry::ConstPtr& trackingcamera_odom_msg)
{ 
  try
  {
    _handler->trackingcamera_T_basefootprint_ = tf2::transformToEigen(_handler->tf_buffer_.lookupTransform(_trackingcamera_frame_id,_base_footprint_frame_id,ros::Time(0)));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  // The tracking camera provides us the following transform: odom_T_trackingcamera
  // We need to report everything wrt the robot's base footprint i.e.: odom_T_basefootprint = odom_T_trackingcamera * trackingcamera_T_basefootprint
  _handler->odom_T_trackingcamera_.translation() = Eigen::Vector3d(trackingcamera_odom_msg->pose.pose.position.x,trackingcamera_odom_msg->pose.pose.position.y,trackingcamera_odom_msg->pose.pose.position.z);
  _handler->odom_T_trackingcamera_.linear() = Eigen::Quaterniond(trackingcamera_odom_msg->pose.pose.orientation.w,trackingcamera_odom_msg->pose.pose.orientation.x,trackingcamera_odom_msg->pose.pose.orientation.y,trackingcamera_odom_msg->pose.pose.orientation.z).toRotationMatrix();
  _handler->odom_T_basefootprint_ = _handler->odom_T_trackingcamera_ * _handler->trackingcamera_T_basefootprint_;

  // Get current ROS time
  _t = ros::Time::now();

  // Publish TF msg
  // The navigation stack uses tf to determine the robot's location in the world and relate sensor data to a static map.
  _handler->transform_msg_ = tf2::eigenToTransform(_handler->odom_T_basefootprint_);
  _handler->transform_msg_.header.seq++;
  _handler->transform_msg_.header.stamp = _t;
  _handler->transform_msg_.header.frame_id = _odom_frame_id;
  _handler->transform_msg_.child_frame_id = _base_footprint_frame_id;
  _handler->tf_broadcaster_.sendTransform(_handler->transform_msg_);

  if(_t != _t_prev)
  {
    // Publish odom msg
    // The nav_msgs/Odometry message stores an estimate of the position and velocity of a robot in free space:
    // This represents an estimate of a position and velocity in free space.
    // The pose in this message should be specified in the coordinate frame given by header.frame_id (in our case odom)
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id (in our case the basefootprint)
    _handler->odom_msg_.header                = _handler->transform_msg_.header;
    _handler->odom_msg_.child_frame_id        = _handler->transform_msg_.child_frame_id;
    _handler->odom_msg_.pose.pose.position.x  = _handler->transform_msg_.transform.translation.x;
    _handler->odom_msg_.pose.pose.position.y  = _handler->transform_msg_.transform.translation.y;
    _handler->odom_msg_.pose.pose.position.z  = _handler->transform_msg_.transform.translation.z;
    _handler->odom_msg_.pose.pose.orientation = _handler->transform_msg_.transform.rotation;

    if(_twist_in_odom_frame) // In this case the twist from the trackingcamera is defined wrt the odom frame
    {
      // basefootprint_v = basefootprint_R_odom * odom_v_trackingcamera + basefootprint_S_odom * basefootprint_R_odom * odom_p_trackingcamera
      // basefootprint_omega = basefootprint_R_odom * odom_omega_trackingcamera

      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.linear, _handler->trackingcamera_v_);
      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.angular,_handler->trackingcamera_omega_);

      _handler->basefootprint_R_odom_ = _handler->odom_T_basefootprint_.linear().transpose();

      _handler->basefootprint_v_ = _handler->basefootprint_R_odom_ * _handler->trackingcamera_v_; // TODO + basefootprint_S_odom * basefootprint_R_odom * odom_p_trackingcamera
      _handler->basefootprint_omega_ = _handler->basefootprint_R_odom_ * _handler->trackingcamera_omega_;

      tf2::toMsg(_handler->basefootprint_v_,_handler->odom_msg_.twist.twist.linear);
      tf2::toMsg(_handler->basefootprint_omega_,_handler->odom_msg_.twist.twist.angular);
    }
    else  // In this case the twist from the trackingcamera is defined wrt to itself
    {
      // basefootprint_v = basefootprint_R_trackingcamera * trackingcamera_v + basefootprint_S_trackingcamera * basefootprint_R_trackingcamera * trackingcamera_p
      // basefootprint_omega = basefootprint_R_trackingcamera * trackingcamera_omega

      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.linear, _handler->trackingcamera_v_);
      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.angular,_handler->trackingcamera_omega_);

      _handler->basefootprint_R_trackingcamera_ = _handler->trackingcamera_T_basefootprint_.linear().transpose();

      _handler->basefootprint_v_ = _handler->basefootprint_R_trackingcamera_ * _handler->trackingcamera_v_; // TODO + basefootprint_S_trackingcamera * basefootprint_R_trackingcamera * trackingcamera_p
      _handler->basefootprint_omega_ = _handler->basefootprint_R_trackingcamera_ * _handler->trackingcamera_omega_;

      tf2::toMsg(_handler->basefootprint_v_,_handler->odom_msg_.twist.twist.linear);
      tf2::toMsg(_handler->basefootprint_omega_,_handler->odom_msg_.twist.twist.angular);
    }
    //_handler->odom_msg_.twist                  = odom_T_trackingcamera->twist;
    //_handler->odom_msg_.twist.twist.linear.z   = 0.0; // basefootprint is on the ground
    //_handler->odom_msg_.twist.twist.angular.x  = 0.0; // no roll
    //_handler->odom_msg_.twist.twist.angular.y  = 0.0; // no pitch
    _handler->odom_msg_.twist.covariance      = trackingcamera_odom_msg->twist.covariance; // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
    _handler->odom_msg_.pose.covariance       = trackingcamera_odom_msg->pose.covariance; // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
    _handler->odom_publisher_.publish(_handler->odom_msg_);
  }
  _t_prev = _t;
}


int main(int argc, char** argv)
{
  std::string ns = "odometry_publisher";

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  // get ROS params
  std::string output_topic;
  std::vector<std::string> trackingcamera_topics;
  n.getParam("trackingcamera_topics", trackingcamera_topics);
  n.getParam("output_topic", output_topic);
  n.getParam("trackingcamera_frame_id", _trackingcamera_frame_id);
  n.getParam("basefootprint_frame_id", _base_footprint_frame_id);
  n.getParam("odom_frame_id", _odom_frame_id);
  n.getParam("twist_in_odom_frame", _twist_in_odom_frame);

  //std::vector<std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> > > trackingcamera_odom_subs(trackingcamera_topics.size());
  //for(unsigned int i=0;i<trackingcamera_topics.size();i++)
  //  trackingcamera_odom_subs[i].reset(new message_filters::Subscriber<nav_msgs::Odometry>(n,trackingcamera_topics[i],20));

  message_filters::Syncronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);

  switch(trackingcamera_topics.size())
  {
    case 1:

    break;

    case 2:
    break;


  };


  //sync.registerCallback(boost::bind(&callback, _1, _2));


  _t = _t_prev = ros::Time::now();

  _handler = std::make_shared<RosTransformHandler>(n);

  // initialize trackingcamera odom subscriber
  //ros::Subscriber trackingcamera_sub = n.subscribe(trackingcamera_topic, 20, callback);

  ros::spin();

  return 0;
}
