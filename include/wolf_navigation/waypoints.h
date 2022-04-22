#ifndef WOLF_NAVIGATION_WAYPOINTS_H
#define WOLF_NAVIGATION_WAYPOINTS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/utils.h>

#include <deque>
#include <mutex>

#include "wolf_navigation/waypoint.h"

class Waypoints
{

public:

  const std::string CLASS_NAME = "Waypoints";

  typedef std::shared_ptr<Waypoints> Ptr;

  Waypoints(ros::NodeHandle& nh);

  ~Waypoints()  {}

  void addWaypoint(const move_base_msgs::MoveBaseGoal& waypoint);

  void addWaypoint(const geometry_msgs::PoseStamped& waypoint);

  void addWaypoint(const geometry_msgs::PointStamped& waypoint);

  move_base_msgs::MoveBaseGoal getCurrentWaypointGoal();

  unsigned int getCurrentWaypointId();

  void removeCurrentWaypoint();

  void removeWaypoint(const unsigned int& id);

  void moveToNextWaypoint();

  unsigned int getNumberOfWaypoints();

private:
  ros::NodeHandle& nh_;
  std::deque<Waypoint::Ptr> list_;
  std::mutex mtx_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
};

#endif
