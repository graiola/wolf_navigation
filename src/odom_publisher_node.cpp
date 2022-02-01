#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static geometry_msgs::TransformStamped _odom_trans;
static ros::Time _prev_stamp{0.0};

void convertOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
  //first, we'll publish the transform over tf
  _odom_trans.header.stamp = odom->header.stamp;
  _odom_trans.header.frame_id = "odom";
  _odom_trans.child_frame_id = "base_footprint";

  _odom_trans.transform.translation.x = odom->pose.pose.position.x;
  _odom_trans.transform.translation.y = odom->pose.pose.position.y;
  _odom_trans.transform.translation.z = 0; // not set to odom->pose.pose.position.z because during stand up the robot fly
  _odom_trans.transform.rotation = odom->pose.pose.orientation;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("t265/odom", 20, convertOdom);
  tf::TransformBroadcaster odom_broadcaster;



  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();

    if (_prev_stamp != _odom_trans.header.stamp)
    {
      //send the transform
      odom_broadcaster.sendTransform(_odom_trans);

      _prev_stamp = _odom_trans.header.stamp;
    }

    r.sleep();
  }
}
