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

RosTransformHandler::Ptr _data;

// The tracking camera provides us the following transform odom_T_tracking_camera
// we need to create: odom_T_basefoot_print = odom_T_tracking_camera * tracking_camera_T_basefoot_print
void callback(const nav_msgs::Odometry::ConstPtr& odom_T_tracking_camera)
{

  try
  {
    //auto T = _data->_tf_buffer.lookupTransform(_base_footprint_frame_id,_tracking_camera_frame_id,ros::Time(0));
    //_data->_tracking_camera_T_basefoot_print.setOrigin(tf2::Vector3(T.transform.translation.x,T.transform.translation.y,T.transform.translation.z));
    //_data->_tracking_camera_T_basefoot_print.setRotation(tf2::Quaternion(T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z,T.transform.rotation.w));
    tf2::fromMsg(_data->_tf_buffer.lookupTransform(_base_footprint_frame_id,_tracking_camera_frame_id,ros::Time(0)),_data->_tracking_camera_T_basefoot_print);

    //_data->_tracking_camera_T_basefoot_print.setOrigin(tf2::Vector3(_data->_tracking_camera_T_basefoot_print.getOrigin().getX(),
    //                                                                _data->_tracking_camera_T_basefoot_print.getOrigin().getY(),
    //                                                                _data->_tracking_camera_T_basefoot_print.getOrigin().getX());
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  _data->_odom_T_tracking_camera.setOrigin(tf2::Vector3(odom_T_tracking_camera->pose.pose.position.x,odom_T_tracking_camera->pose.pose.position.y,odom_T_tracking_camera->pose.pose.position.z));
  _data->_odom_T_tracking_camera.setRotation(tf2::Quaternion(odom_T_tracking_camera->pose.pose.orientation.x,odom_T_tracking_camera->pose.pose.orientation.y,odom_T_tracking_camera->pose.pose.orientation.z,odom_T_tracking_camera->pose.pose.orientation.w));

  //_odom_T_basefoot_print.transform.translation.x = odom->pose.pose.position.x;
  //_odom_T_basefoot_print.transform.translation.y = odom->pose.pose.position.y;
  //_odom_T_basefoot_print.transform.translation.z = 0; // not set to odom->pose.pose.position.z because during stand up the robot fly
  //_odom_T_basefoot_print.transform.rotation = odom->pose.pose.orientation;

  _data->_odom_T_basefoot_print.mult(_data->_odom_T_tracking_camera,_data->_tracking_camera_T_basefoot_print);

  //geometry_msgs::TransformStamped T;
  //T.transform  = tf2::toMsg(_data->_odom_T_basefoot_print.inverse());
  //T.header.seq++;
  //T.header.stamp = ros::Time::now();
  //T.header.frame_id = _base_footprint_frame_id;
  //T.child_frame_id = _odom_frame_id;

  auto T = tf2::toMsg(_data->_odom_T_basefoot_print);
  T.header.seq++;
  T.header.stamp = ros::Time::now();
  T.header.frame_id = _odom_frame_id;
  T.child_frame_id = _base_footprint_frame_id;

  _data->_br.sendTransform(T);
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

  _data = std::make_shared<RosTransformHandler>();

  // initialize odom subscriber
  ros::Subscriber tracking_camera_sub = n.subscribe(tracking_camera_topic, 20, callback);

  ros::spin();
}
