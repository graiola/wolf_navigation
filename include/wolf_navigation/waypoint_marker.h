#ifndef WOLF_NAVIGATION_WAYPOINT_MARKER_H
#define WOLF_NAVIGATION_WAYPOINT_MARKER_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/utils.h>

class WaypointMarker
{

public:

  typedef std::shared_ptr<WaypointMarker> Ptr;

  WaypointMarker(ros::NodeHandle& nh)
    :nh_(nh)
  {
      pub_ = nh_.advertise<visualization_msgs::Marker>( "waypoint_markers", 0 );
  }

  ~WaypointMarker()
  {

  }

  void makeMarker(const unsigned int& id, const move_base_msgs::MoveBaseGoal& waypoint)
  {

    visualization_msgs::Marker marker;
    marker.header.frame_id = waypoint.target_pose.header.frame_id;
    marker.header.stamp = waypoint.target_pose.header.stamp;
    marker.ns = "wolf_navigation";
    marker.id = id;
    marker.frame_locked = 1;
    marker.type = marker.MESH_RESOURCE;
    marker.action = marker.ADD;
    marker.mesh_use_embedded_materials = true;
    marker.pose.position.x = waypoint.target_pose.pose.position.x;
    marker.pose.position.y = waypoint.target_pose.pose.position.y;
    marker.pose.position.z = waypoint.target_pose.pose.position.z;
    marker.pose.orientation.x = waypoint.target_pose.pose.orientation.x;
    marker.pose.orientation.y = waypoint.target_pose.pose.orientation.y;
    marker.pose.orientation.z = waypoint.target_pose.pose.orientation.z;
    marker.pose.orientation.w = waypoint.target_pose.pose.orientation.w;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.mesh_resource = "package://wolf_navigation/media/waypoint.dae";
    pub_.publish( marker );
  }

private:

  visualization_msgs::Marker marker_;
  ros::Publisher pub_;
  ros::NodeHandle nh_;

};

#endif
