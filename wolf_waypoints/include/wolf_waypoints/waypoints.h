#ifndef WOLF_WAYPOINTS_WAYPOINTS_H
#define WOLF_WAYPOINTS_WAYPOINTS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base/move_base.h>
#include <tf2/utils.h>

#include <deque>
#include <mutex>

#include "wolf_waypoints/waypoint.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Waypoints
{

public:

  const std::string CLASS_NAME = "Waypoints";

  typedef std::shared_ptr<Waypoints> Ptr;

  Waypoints(ros::NodeHandle& nh, std::shared_ptr<MoveBaseClient> move_base);

  ~Waypoints()  {}

  void addWaypoint(const move_base_msgs::MoveBaseGoal& waypoint);

  void addWaypoint(const geometry_msgs::PoseStamped& waypoint);

  void addWaypoint(const geometry_msgs::PointStamped& waypoint);

  move_base_msgs::MoveBaseGoal getCurrentWaypointGoal();

  int getCurrentWaypointId();

  void cancelGoal();

  void cancelAllGoals();

  void removeCurrentWaypoint();

  void removeWaypoint(const int &id);

  void removeAllWaypoints();

  void moveToNextWaypoint();

  unsigned int getNumberOfWaypoints();

private:

  Waypoint *getWaypoint(const int& idx) const;

  ros::NodeHandle& nh_;
  std::deque<Waypoint::Ptr> list_;
  std::mutex mtx_;

  std::shared_ptr<MoveBaseClient> move_base_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
};

#endif
