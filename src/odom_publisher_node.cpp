#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>

static std::string _trackingcamera_frame_id;
static std::string _base_footprint_frame_id;
static std::string _odom_frame_id;

struct RosTransformHandler {

    typedef std::shared_ptr<RosTransformHandler> Ptr;

    RosTransformHandler(ros::NodeHandle& n, const std::string& odom_topic = "/odom"): tf_listener_(tf_buffer_) {odom_publisher_ = n.advertise<nav_msgs::Odometry>(odom_topic,20);}

    Eigen::Isometry3d trackingcamera_T_basefootprint_;
    Eigen::Isometry3d odom_T_trackingcamera_;
    Eigen::Isometry3d odom_T_basefootprint_;
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped transform_msg_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Publisher odom_publisher_;
};

static RosTransformHandler::Ptr _handler;
static ros::Time _t;

// The tracking camera provides us the following transform odom_T_trackingcamera
// we need to create: odom_T_basefootprint = odom_T_trackingcamera * trackingcamera_T_basefootprint
void callback(const nav_msgs::Odometry::ConstPtr& odom_T_trackingcamera)
{ 
  try
  {
    _handler->trackingcamera_T_basefootprint_ = tf2::transformToEigen(_handler->tf_buffer_.lookupTransform(_trackingcamera_frame_id,_base_footprint_frame_id,ros::Time(0)));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  _handler->odom_T_trackingcamera_.translation() = Eigen::Vector3d(odom_T_trackingcamera->pose.pose.position.x,odom_T_trackingcamera->pose.pose.position.y,odom_T_trackingcamera->pose.pose.position.z);
  _handler->odom_T_trackingcamera_.linear() = Eigen::Quaterniond(odom_T_trackingcamera->pose.pose.orientation.w,odom_T_trackingcamera->pose.pose.orientation.x,odom_T_trackingcamera->pose.pose.orientation.y,odom_T_trackingcamera->pose.pose.orientation.z).toRotationMatrix();
  _handler->odom_T_basefootprint_ = _handler->odom_T_trackingcamera_ *_handler->trackingcamera_T_basefootprint_;

  // Get current ROS time
  _t = ros::Time::now();

  // Publish TF msg
  _handler->transform_msg_ = tf2::eigenToTransform(_handler->odom_T_basefootprint_);
  _handler->transform_msg_.header.seq++;
  _handler->transform_msg_.header.stamp = _t;
  _handler->transform_msg_.header.frame_id = _odom_frame_id;
  _handler->transform_msg_.child_frame_id = _base_footprint_frame_id;
  _handler->tf_broadcaster_.sendTransform(_handler->transform_msg_);

  // Publish odom msg
  _handler->odom_msg_.header                = _handler->transform_msg_.header;
  _handler->odom_msg_.child_frame_id        = _handler->transform_msg_.child_frame_id;
  _handler->odom_msg_.pose.pose.position.x  = _handler->transform_msg_.transform.translation.x;
  _handler->odom_msg_.pose.pose.position.y  = _handler->transform_msg_.transform.translation.y;
  _handler->odom_msg_.pose.pose.position.z  = _handler->transform_msg_.transform.translation.z;
  _handler->odom_msg_.pose.pose.orientation = _handler->transform_msg_.transform.rotation;

  // odom_v_basefootprint = basefootprint_R_trackingcamera * v_trackingcamera + basefootprint_S_trackingcamera * p_trackingcamera
  // odom_omega_basefootprint = basefootprint_R_trackingcamera * omega_trackingcamera
  _handler->odom_msg_.twist                 = odom_T_trackingcamera->twist;  // FIXME
  _handler->odom_msg_.pose.covariance       = odom_T_trackingcamera->pose.covariance; // Use the same covariance of the tracking camera
  _handler->odom_publisher_.publish(_handler->odom_msg_);
}


int main(int argc, char** argv)
{
  std::string ns = "odometry_publisher";

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  // get tracking camera subscriber topic
  std::string trackingcamera_topic, output_topic;
  n.getParam("trackingcamera_topic", trackingcamera_topic);
  n.getParam("output_topic", output_topic);
  n.getParam("trackingcamera_frame_id", _trackingcamera_frame_id);
  n.getParam("basefootprint_frame_id", _base_footprint_frame_id);
  n.getParam("odom_frame_id", _odom_frame_id);

  _handler = std::make_shared<RosTransformHandler>(n);

  // initialize odom subscriber
  ros::Subscriber trackingcamera_sub = n.subscribe(trackingcamera_topic, 20, callback);

  ros::spin();

  return 0;
}
