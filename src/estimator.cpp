#include "wolf_navigation/estimator.h"

wolf_navigation::EstimatorInterface::EstimatorInterface(ros::NodeHandle &n, const std::string &odom_topic)
  : tf_listener_(tf_buffer_)
{
  odom_publisher_ = n.advertise<nav_msgs::Odometry>(odom_topic,20);
}

void wolf_navigation::TrackingCamera::update(const nav_msgs::Odometry::ConstPtr& trackingcamera_odom_msg)
{
  try
  {
    trackingcamera_T_basefootprint_ = tf2::transformToEigen(tf_buffer_.lookupTransform(_trackingcamera_frame_id,_base_footprint_frame_id,ros::Time(0)));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  // The tracking camera provides us the following transform: odom_T_trackingcamera
  // We need to report everything wrt the robot's base footprint i.e.: odom_T_basefootprint = odom_T_trackingcamera * trackingcamera_T_basefootprint
  odom_T_trackingcamera_.translation() = Eigen::Vector3d(trackingcamera_odom_msg->pose.pose.position.x,trackingcamera_odom_msg->pose.pose.position.y,trackingcamera_odom_msg->pose.pose.position.z);
  odom_T_trackingcamera_.linear() = Eigen::Quaterniond(trackingcamera_odom_msg->pose.pose.orientation.w,trackingcamera_odom_msg->pose.pose.orientation.x,trackingcamera_odom_msg->pose.pose.orientation.y,trackingcamera_odom_msg->pose.pose.orientation.z).toRotationMatrix();
  odom_T_basefootprint_ = odom_T_trackingcamera_ * trackingcamera_T_basefootprint_;

  // Get current ROS time
  t_ = ros::Time::now();

  // Publish TF msg
  // The navigation stack uses tf to determine the robot's location in the world and relate sensor data to a static map.
  transform_msg_ = tf2::eigenToTransform(odom_T_basefootprint_);
  transform_msg_.header.seq++;
  transform_msg_.header.stamp = _t;
  transform_msg_.header.frame_id = _odom_frame_id;
  transform_msg_.child_frame_id = _base_footprint_frame_id;
  tf_broadcaster_.sendTransform(transform_msg_);

  if(t_ != t_prev_)
  {
    // Publish odom msg
    // The nav_msgs/Odometry message stores an estimate of the position and velocity of a robot in free space:
    // This represents an estimate of a position and velocity in free space.
    // The pose in this message should be specified in the coordinate frame given by header.frame_id (in our case odom)
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id (in our case the basefootprint)
    odom_msg_.header                = transform_msg_.header;
    odom_msg_.child_frame_id        = transform_msg_.child_frame_id;
    odom_msg_.pose.pose.position.x  = transform_msg_.transform.translation.x;
    odom_msg_.pose.pose.position.y  = transform_msg_.transform.translation.y;
    odom_msg_.pose.pose.position.z  = transform_msg_.transform.translation.z;
    odom_msg_.pose.pose.orientation = transform_msg_.transform.rotation;

    if(_twist_in_odom_frame) // In this case the twist from the trackingcamera is defined wrt the odom frame
    {
      // basefootprint_v = basefootprint_R_odom * odom_v_trackingcamera + basefootprint_S_odom * basefootprint_R_odom * odom_p_trackingcamera
      // basefootprint_omega = basefootprint_R_odom * odom_omega_trackingcamera

      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.linear, trackingcamera_v_);
      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.angular,trackingcamera_omega_);

      basefootprint_R_odom_ = odom_T_basefootprint_.linear().transpose();

      basefootprint_v_ = basefootprint_R_odom_ * trackingcamera_v_; // TODO + basefootprint_S_odom * basefootprint_R_odom * odom_p_trackingcamera
      basefootprint_omega_ = basefootprint_R_odom_ * trackingcamera_omega_;

      tf2::toMsg(basefootprint_v_,odom_msg_.twist.twist.linear);
      tf2::toMsg(basefootprint_omega_,odom_msg_.twist.twist.angular);
    }
    else  // In this case the twist from the trackingcamera is defined wrt to itself
    {
      // basefootprint_v = basefootprint_R_trackingcamera * trackingcamera_v + basefootprint_S_trackingcamera * basefootprint_R_trackingcamera * trackingcamera_p
      // basefootprint_omega = basefootprint_R_trackingcamera * trackingcamera_omega

      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.linear, trackingcamera_v_);
      tf2::fromMsg(trackingcamera_odom_msg->twist.twist.angular,trackingcamera_omega_);

      basefootprint_R_trackingcamera_ = trackingcamera_T_basefootprint_.linear().transpose();

      basefootprint_v_ = basefootprint_R_trackingcamera_ * trackingcamera_v_; // TODO + basefootprint_S_trackingcamera * basefootprint_R_trackingcamera * trackingcamera_p
      basefootprint_omega_ = basefootprint_R_trackingcamera_ * trackingcamera_omega_;

      tf2::toMsg(basefootprint_v_,odom_msg_.twist.twist.linear);
      tf2::toMsg(basefootprint_omega_,odom_msg_.twist.twist.angular);
    }
    //odom_msg_.twist                  = odom_T_trackingcamera->twist;
    //odom_msg_.twist.twist.linear.z   = 0.0; // basefootprint is on the ground
    //odom_msg_.twist.twist.angular.x  = 0.0; // no roll
    //odom_msg_.twist.twist.angular.y  = 0.0; // no pitch
    odom_msg_.twist.covariance      = trackingcamera_odom_msg->twist.covariance; // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
    odom_msg_.pose.covariance       = trackingcamera_odom_msg->pose.covariance; // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
    odom_publisher_.publish(odom_msg_);
  }
  t_prev_ = t_;
}
