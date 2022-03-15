#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>



static std::string _tracking_camera_frame_id;
static std::string _base_footprint_frame_id;
static std::string _odom_frame_id;
static tf2::Stamped<tf2::Transform> _tracking_camera_T_basefoot_print;
static tf2::Stamped<tf2::Transform> _odom_T_tracking_camera;
static tf2::Stamped<tf2::Transform> _odom_T_basefoot_print;
static tf2_ros::Buffer _tf_buffer;
static tf2_ros::TransformListener _tf_listener(_tf_buffer);

// The tracking camera provides us the following transform odom_T_tracking_camera
// we need to create: odom_T_basefoot_print = odom_T_tracking_camera * tracking_camera_T_basefoot_print

void callback(const nav_msgs::Odometry::ConstPtr& odom_T_tracking_camera)
{

  //_odom_T_basefoot_print.header.stamp    = ros::Time::now();
  //_odom_T_basefoot_print.header.frame_id = _odom_frame_id;
  //_odom_T_basefoot_print.child_frame_id  = _base_footprint_frame_id;

  try
  {
    _tracking_camera_T_basefoot_print. = _tf_buffer.lookupTransform(_base_footprint_frame_id,_tracking_camera_frame_id,ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  _odom_T_tracking_camera.transform.translation.x = odom_T_tracking_camera->pose.pose.position.x;
  _odom_T_tracking_camera.transform.translation.y = odom_T_tracking_camera->pose.pose.position.y;
  _odom_T_tracking_camera.transform.translation.z = odom_T_tracking_camera->pose.pose.position.z;

  _odom_T_tracking_camera.transform.rotation.x = odom_T_tracking_camera->pose.pose.orientation.x;
  _odom_T_tracking_camera.transform.rotation.y = odom_T_tracking_camera->pose.pose.orientation.y;
  _odom_T_tracking_camera.transform.rotation.z = odom_T_tracking_camera->pose.pose.orientation.z;
  _odom_T_tracking_camera.transform.rotation.w = odom_T_tracking_camera->pose.pose.orientation.w;



  //_odom_T_basefoot_print.transform.translation.x = odom->pose.pose.position.x;
  //_odom_T_basefoot_print.transform.translation.y = odom->pose.pose.position.y;
  //_odom_T_basefoot_print.transform.translation.z = 0; // not set to odom->pose.pose.position.z because during stand up the robot fly
  //_odom_T_basefoot_print.transform.rotation = odom->pose.pose.orientation;


  _odom_T_basefoot_print.mult(_odom_T_tracking_camera,_tracking_camera_T_basefoot_print);
}


int main(int argc, char** argv)
{
  std::string ns{"odometry_publisher"};

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  // get tracking camera subscriber topic
  std::string tracking_camera_topic, output_topic;
  n.getParam("tracking_camera_topic", tracking_camera_topic);
  n.getParam("output_topic", output_topic);
  n.getParam("tracking_camera_frame_id", _tracking_camera_frame_id);
  n.getParam("base_footprint_frame_id", _base_footprint_frame_id);

  // initialize odom subscriber
  ros::Subscriber tracking_camera_sub = n.subscribe(tracking_camera_topic, 20, callback);

  ros::spin();
}
