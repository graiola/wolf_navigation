#include "wolf_waypoints/waypoint.h"

Waypoint::Waypoint(const unsigned int& id, const move_base_msgs::MoveBaseGoal& goal)
{
  name_ = "waypoint_"+std::to_string(id);
  goal_ = goal;
  id_   = id;

  createMarker(id,goal);

  menu_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  menu_control_.always_visible = true;
  menu_control_.markers.push_back( marker_ );

  createInteractiveMarker(marker_);
}

visualization_msgs::InteractiveMarker Waypoint::getInteractiveMarker()
{
  return interactive_marker_;
}

const std::string& Waypoint::getName()
{
  return name_;
}

const int& Waypoint::getId()
{
  return id_;
}

const move_base_msgs::MoveBaseGoal& Waypoint::getGoal()
{
  return goal_;
}

void Waypoint::createMenuEntry(const interactive_markers::MenuHandler::FeedbackCallback& f, interactive_markers::InteractiveMarkerServer &interactive_marker_server)
{
  auto remove_entry = menu_handler_.insert("remove "+name_,f);
  menu_handler_.setVisible(remove_entry,true);
  menu_handler_.apply(interactive_marker_server,interactive_marker_.name);
}

void Waypoint::createInteractiveMarker(const visualization_msgs::Marker& marker)
{
  interactive_marker_.name = name_;
  interactive_marker_.pose = marker.pose;
  interactive_marker_.header = marker.header;
  interactive_marker_.controls.push_back(menu_control_);
}

void Waypoint::createMarker(const unsigned int& id, const move_base_msgs::MoveBaseGoal& waypoint)
{
  marker_.header.frame_id = waypoint.target_pose.header.frame_id;
  marker_.header.stamp = waypoint.target_pose.header.stamp;
  marker_.ns = "wolf_navigation"; // NOTE: hardcoded
  marker_.id = id;
  marker_.frame_locked = 1;
  marker_.type = marker_.MESH_RESOURCE;
  marker_.action = marker_.ADD;
  marker_.mesh_use_embedded_materials = true;
  marker_.pose.position.x = waypoint.target_pose.pose.position.x;
  marker_.pose.position.y = waypoint.target_pose.pose.position.y;
  marker_.pose.position.z = waypoint.target_pose.pose.position.z;
  marker_.pose.orientation.x = waypoint.target_pose.pose.orientation.x;
  marker_.pose.orientation.y = waypoint.target_pose.pose.orientation.y;
  marker_.pose.orientation.z = waypoint.target_pose.pose.orientation.z;
  marker_.pose.orientation.w = waypoint.target_pose.pose.orientation.w;
  marker_.scale.x = 0.25;
  marker_.scale.y = 0.25;
  marker_.scale.z = 0.25;
  marker_.color.a = 1.0;
  marker_.mesh_resource = "package://wolf_waypoints/media/waypoint.dae";
}



