#include "wolf_navigation/ros_wrapper.h"

using namespace wolf_navigation;

RosWrapper::RosWrapper(ros::NodeHandle &nh)
    : tf_listener_(tf_buffer_)
{
    t_ = t_prev_ = ros::Time::now();
    nh_ = nh;
}

void RosWrapper::init(const std::string& trackingcamera_topic, const std::string& child_frame_id)
{
    trackingcamera_topic_ = trackingcamera_topic;

    child_frame_id_ = child_frame_id;
    odom_frame_id_ = "odom";

    sub_ = nh_.subscribe(trackingcamera_topic_,20,&RosWrapper::callback,this);
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_,20);
}

void RosWrapper::callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    try
    {
        single_tc_.setTransform(tf2::transformToEigen(tf_buffer_.lookupTransform(odom_msg->child_frame_id,child_frame_id_,ros::Time(0))));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }

    tmp_isometry3d_.translation() = Eigen::Vector3d(odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z);
    tmp_isometry3d_.linear() = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z).toRotationMatrix();

    tf2::fromMsg(odom_msg->twist.twist.linear, tmp_vector3d_);
    tf2::fromMsg(odom_msg->twist.twist.angular,tmp_vector3d_1_);

    single_tc_.setLinearTwist(tmp_vector3d_);
    single_tc_.setAngularTwist(tmp_vector3d_1_);
    single_tc_.setPose(tmp_isometry3d_);

    single_tc_.update();

    // Get current ROS time
    t_ = ros::Time::now();

    if(t_ != t_prev_)
    {
        // Publish TF msg
        // The navigation stack uses tf to determine the robot's location in the world and relate sensor data to a static map.
        transform_msg_out_ = tf2::eigenToTransform(single_tc_.getPose());
        transform_msg_out_.header.seq++;
        transform_msg_out_.header.stamp = t_;
        transform_msg_out_.header.frame_id = odom_frame_id_;
        transform_msg_out_.child_frame_id = child_frame_id_;
        tf_broadcaster_.sendTransform(transform_msg_out_);

        // Publish odom msg
        // The nav_msgs/Odometry message stores an estimate of the position and velocity of a robot in free space:
        // This represents an estimate of a position and velocity in free space.
        // The pose in this message should be specified in the coordinate frame given by header.frame_id (in our case odom)
        // The twist in this message should be specified in the coordinate frame given by the child_frame_id (in our case the basefootprint)
        odom_msg_out_.header                = transform_msg_out_.header;
        odom_msg_out_.child_frame_id        = transform_msg_out_.child_frame_id;
        odom_msg_out_.pose.pose.position.x  = transform_msg_out_.transform.translation.x;
        odom_msg_out_.pose.pose.position.y  = transform_msg_out_.transform.translation.y;
        odom_msg_out_.pose.pose.position.z  = transform_msg_out_.transform.translation.z;
        odom_msg_out_.pose.pose.orientation = transform_msg_out_.transform.rotation;

        tf2::toMsg(single_tc_.getLinearTwist(),odom_msg_out_.twist.twist.linear);
        tf2::toMsg(single_tc_.getAngularTwist(),odom_msg_out_.twist.twist.angular);

        //odom_msg_.twist                  = odom_T_trackingcamera->twist;
        //odom_msg_.twist.twist.linear.z   = 0.0; // basefootprint is on the ground
        //odom_msg_.twist.twist.angular.x  = 0.0; // no roll
        //odom_msg_.twist.twist.angular.y  = 0.0; // no pitch
        odom_msg_out_.twist.covariance      = odom_msg->twist.covariance; // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
        odom_msg_out_.pose.covariance       = odom_msg->pose.covariance; // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
        odom_publisher_.publish(odom_msg_out_);
    }
    t_prev_ = t_;
}
