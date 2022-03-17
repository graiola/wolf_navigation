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

    RosTransformHandler(): _tf_listener(_tf_buffer) {}

    tf2::Stamped<tf2::Transform> _tracking_camera_T_basefoot_print;
    tf2::Stamped<tf2::Transform> _odom_T_tracking_camera;
    tf2::Stamped<tf2::Transform> _odom_T_basefoot_print;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _br;
};

static RosTransformHandler::Ptr _handler;

// The tracking camera provides us the following transform odom_T_tracking_camera
// we need to create: odom_T_basefoot_print = odom_T_tracking_camera * tracking_camera_T_basefoot_print
void callback(const nav_msgs::Odometry::ConstPtr& odom_T_tracking_camera)
{

  try
  {
    tf2::fromMsg(_handler->_tf_buffer.lookupTransform(_tracking_camera_frame_id,_base_footprint_frame_id,ros::Time(0)),_handler->_tracking_camera_T_basefoot_print);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  _handler->_odom_T_tracking_camera.setOrigin(tf2::Vector3(odom_T_tracking_camera->pose.pose.position.x,odom_T_tracking_camera->pose.pose.position.y,odom_T_tracking_camera->pose.pose.position.z));
  _handler->_odom_T_tracking_camera.setRotation(tf2::Quaternion(odom_T_tracking_camera->pose.pose.orientation.x,odom_T_tracking_camera->pose.pose.orientation.y,odom_T_tracking_camera->pose.pose.orientation.z,odom_T_tracking_camera->pose.pose.orientation.w));

  _handler->_odom_T_basefoot_print.mult(_handler->_odom_T_tracking_camera,_handler->_tracking_camera_T_basefoot_print);

  auto T = tf2::toMsg(_handler->_odom_T_basefoot_print);

  T.header.seq++;
  T.header.stamp = ros::Time::now();
  T.header.frame_id = _odom_frame_id;
  T.child_frame_id = _base_footprint_frame_id;

  _handler->_br.sendTransform(T);
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

  _handler = std::make_shared<RosTransformHandler>();

  // initialize odom subscriber
  ros::Subscriber tracking_camera_sub = n.subscribe(tracking_camera_topic, 20, callback);

  ros::spin();
}
