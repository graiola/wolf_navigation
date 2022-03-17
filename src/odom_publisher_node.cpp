#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

static std::string _tracking_camera_frame_id;
static std::string _base_footprint_frame_id;
static std::string _odom_frame_id;

struct RosTransformHandler {

    typedef std::shared_ptr<RosTransformHandler> Ptr;

    RosTransformHandler(ros::NodeHandle& n, const std::string& odom_topic = "/odom"): tf_listener_(tf_buffer_) {odom_publisher_ = n.advertise<nav_msgs::Odometry>(odom_topic,20);}

    tf2::Stamped<tf2::Transform> tracking_camera_T_basefoot_print_;
    tf2::Stamped<tf2::Transform> odom_T_tracking_camera_;
    tf2::Stamped<tf2::Transform> odom_T_basefoot_print_;
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped transform_msg_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Publisher odom_publisher_;
};

static RosTransformHandler::Ptr _handler;
static ros::Time _t;

// The tracking camera provides us the following transform odom_T_tracking_camera
// we need to create: odom_T_basefoot_print = odom_T_tracking_camera * tracking_camera_T_basefoot_print
void callback(const nav_msgs::Odometry::ConstPtr& odom_T_tracking_camera)
{ 
  try
  {
    tf2::fromMsg(_handler->tf_buffer_.lookupTransform(_tracking_camera_frame_id,_base_footprint_frame_id,ros::Time(0)),_handler->tracking_camera_T_basefoot_print_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  _handler->odom_T_tracking_camera_.setOrigin(tf2::Vector3(odom_T_tracking_camera->pose.pose.position.x,odom_T_tracking_camera->pose.pose.position.y,odom_T_tracking_camera->pose.pose.position.z));
  _handler->odom_T_tracking_camera_.setRotation(tf2::Quaternion(odom_T_tracking_camera->pose.pose.orientation.x,odom_T_tracking_camera->pose.pose.orientation.y,odom_T_tracking_camera->pose.pose.orientation.z,odom_T_tracking_camera->pose.pose.orientation.w));

  _handler->odom_T_basefoot_print_.mult(_handler->odom_T_tracking_camera_,_handler->tracking_camera_T_basefoot_print_);

  // Get current ROS time
  _t = ros::Time::now();

  // Publish TF msg
  _handler->transform_msg_ = tf2::toMsg(_handler->odom_T_basefoot_print_);
  _handler->transform_msg_.header.seq++;
  _handler->transform_msg_.header.stamp = _t;
  _handler->transform_msg_.header.frame_id = _odom_frame_id;
  _handler->transform_msg_.child_frame_id = _base_footprint_frame_id;
  _handler->tf_broadcaster_.sendTransform(_handler->transform_msg_);

  // Publish odom msg
  _handler->odom_msg_.header                = _handler->transform_msg_.header;
  _handler->odom_msg_.pose.pose.position.x  = _handler->transform_msg_.transform.translation.x;
  _handler->odom_msg_.pose.pose.position.y  = _handler->transform_msg_.transform.translation.y;
  _handler->odom_msg_.pose.pose.position.z  = _handler->transform_msg_.transform.translation.z;
  _handler->odom_msg_.pose.pose.orientation = _handler->transform_msg_.transform.rotation;

  // odom_v_basefoot_print = basefoot_print_R_tracking_camera * v_tracking_camera + basefoot_print_S_tracking_camera * p_tracking_camera
  // odom_omega_basefoot_print = basefoot_print_R_tracking_camera * omega_tracking_camera
  _handler->odom_msg_.twist                 = odom_T_tracking_camera->twist;  // FIXME
  _handler->odom_msg_.pose.covariance       = odom_T_tracking_camera->pose.covariance; // Use the same covariance of the tracking camera
  _handler->odom_publisher_.publish(_handler->odom_msg_);
}

int main(int argc, char** argv)
{
  std::string ns = "odometry_publisher";

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  // get tracking camera subscriber topic
  std::string tracking_camera_topic, output_topic;
  n.getParam("tracking_camera_topic", tracking_camera_topic);
  n.getParam("output_topic", output_topic);
  n.getParam("tracking_camera_frame_id", _tracking_camera_frame_id);
  n.getParam("base_footprint_frame_id", _base_footprint_frame_id);
  n.getParam("odom_frame_id", _odom_frame_id);

  _handler = std::make_shared<RosTransformHandler>(n);

  // initialize odom subscriber
  ros::Subscriber tracking_camera_sub = n.subscribe(tracking_camera_topic, 20, callback);

  ros::spin();
}
