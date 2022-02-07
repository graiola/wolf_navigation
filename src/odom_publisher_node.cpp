#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static geometry_msgs::TransformStamped _odom_trans;
static ros::Time _prev_stamp{0.0};
static std::string _odom_topic{""}, _odom_topic_child{""};

void convertOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
  //first, we'll publish the transform over tf
  _odom_trans.header.stamp = odom->header.stamp;
  _odom_trans.header.frame_id = _odom_topic;
  _odom_trans.child_frame_id = _odom_topic_child;

  _odom_trans.transform.translation.x = odom->pose.pose.position.x;
  _odom_trans.transform.translation.y = odom->pose.pose.position.y;
  _odom_trans.transform.translation.z = 0; // not set to odom->pose.pose.position.z because during stand up the robot fly
  _odom_trans.transform.rotation = odom->pose.pose.orientation;
}


int main(int argc, char** argv){
  std::string ns{"odometry_publisher"};

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

//  set odom subscriber topic
  std::string odom_sub_topic;
  n.getParam("odom_sub_topic", odom_sub_topic);

//  set global odom topics
  n.getParam("odom_topic", _odom_topic);
  n.getParam("odom_topic_child", _odom_topic_child);

//  initialize odom subscriber subscriber
  ros::Subscriber odom_sub = n.subscribe(odom_sub_topic, 20, convertOdom);
//  initialize tf odom broadcaster
  tf::TransformBroadcaster odom_broadcaster;

//   set ros rate
  double ros_rate;
  n.getParam("ros_rate", ros_rate);
  ros::Rate r(ros_rate);

  while(n.ok()){

    ros::spinOnce();
//    check if the current read tf has not been already sent
    if (_prev_stamp != _odom_trans.header.stamp)
    {
//       send the transform
      odom_broadcaster.sendTransform(_odom_trans);
//      update the previous odom time
      _prev_stamp = _odom_trans.header.stamp;
    }

    r.sleep();
  }
}
