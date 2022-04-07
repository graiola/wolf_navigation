#ifndef WOLF_NAVIGATION_WAYPOINTS_H
#define WOLF_NAVIGATION_WAYPOINTS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/utils.h>

#include <wolf_navigation/waypoint_marker.h>

#include <deque>
#include <mutex>

class Waypoints
{

public:

  const std::string CLASS_NAME = "Waypoints";

  typedef std::shared_ptr<Waypoints> Ptr;

  typedef std::pair<unsigned int,move_base_msgs::MoveBaseGoal> waypoint_t;

  Waypoints(ros::NodeHandle& nh);

  ~Waypoints()  {}

  void addWaypoint(const move_base_msgs::MoveBaseGoal& waypoint);

  void addWaypoint(const geometry_msgs::PoseStamped& waypoint);

  void addWaypoint(const geometry_msgs::PointStamped& waypoint);

  move_base_msgs::MoveBaseGoal getCurrentWaypoint();

  unsigned int getCurrentWaypointId();

  void removeCurrentWaypoint();

  void removeWaypoint(const unsigned int& id);

  void moveToNextWaypoint();

  unsigned int getNumberOfWaypoints();

private:
  ros::NodeHandle& nh_;
  std::deque<waypoint_t> list_;
  std::mutex mtx_;

  WaypointMarker::Ptr marker_;
};

#endif
