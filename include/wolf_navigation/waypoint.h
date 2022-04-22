#ifndef WOLF_NAVIGATION_WAYPOINT_H
#define WOLF_NAVIGATION_WAYPOINT_H

#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/utils.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <string>
#include <memory>

class Waypoint
{

public:

  const std::string CLASS_NAME = "Waypoint";

  typedef std::shared_ptr<Waypoint> Ptr;

  Waypoint(const unsigned int& id, const move_base_msgs::MoveBaseGoal& goal);

  visualization_msgs::InteractiveMarker getInteractiveMarker();

  const std::string& getName();

  const unsigned int& getId();

  const move_base_msgs::MoveBaseGoal& getGoal();

  void createMenuEntry(const interactive_markers::MenuHandler::FeedbackCallback &f, interactive_markers::InteractiveMarkerServer &interactive_marker_server);

private:

  void createInteractiveMarker(const visualization_msgs::Marker& marker);

  void createMarker(const unsigned int& id, const move_base_msgs::MoveBaseGoal& waypoint);

  std::string name_;
  unsigned int id_;
  move_base_msgs::MoveBaseGoal goal_;
  interactive_markers::MenuHandler menu_handler_;
  visualization_msgs::Marker marker_;
  visualization_msgs::InteractiveMarkerControl menu_control_;
  visualization_msgs::InteractiveMarker interactive_marker_;
};

#endif
