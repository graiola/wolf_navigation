/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola, Federico Rollo
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

/*
 * @brief follow_waypoints_node.cpp
 * A simple interface to send a sequence of waypoints to move_base
 */

#include <actionlib/client/simple_action_client.h>
#include <rt_gui/rt_gui_client.h>

#include "wolf_navigation/waypoints.h"

using namespace rt_gui;

bool _running = false;
bool _patrol_mode = false;
double _wait_duration = 360.0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::shared_ptr<MoveBaseClient> _move_base;
Waypoints::Ptr _waypoints;

#define NODE_NAME "follow_waypoints_node"

void callback(const geometry_msgs::PoseStamped& waypoint)
{
  _waypoints->addWaypoint(waypoint);
}

void loop()
{

  while(ros::ok())
  {
    if(_running && _waypoints->getNumberOfWaypoints())
    {
      auto goal    = _waypoints->getCurrentWaypoint();
      auto goal_id = _waypoints->getCurrentWaypointId();

      ROS_INFO_NAMED(NODE_NAME,"The robot is moving toward waypoint %i",goal_id);

      _move_base->sendGoal(goal);

      _move_base->waitForResult(ros::Duration(_wait_duration));
      if (_move_base->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        if(_patrol_mode)
          _waypoints->moveToNextWaypoint();
        else
          _waypoints->removeCurrentWaypoint();
        ROS_INFO_NAMED(NODE_NAME,"The robot reached waypoint %i",goal_id);
      }
      else
        ROS_INFO_NAMED(NODE_NAME,"The robot failed to reach waypoint %i",goal_id);

      ros::Duration(0.5).sleep();

    }
  }
}

int main(int argc, char** argv)
{
  std::string ns = NODE_NAME;

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  _move_base.reset(new MoveBaseClient("move_base", true));

  _waypoints.reset(new Waypoints(n));

  ros::Subscriber sub = n.subscribe("waypoints", 1000, callback);

  // create interface
  if(RtGuiClient::getIstance().init("wolf_panel"))
  {
    RtGuiClient::getIstance().addBool(std::string("follow_waypoints"),std::string("Start"),&_running);
    RtGuiClient::getIstance().addBool(std::string("follow_waypoints"),std::string("Patrol mode"),&_patrol_mode);
  }

  std::thread navigation_loop(loop);

  ros::Rate rate(50.0);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();
    rate.sleep();
  }

  navigation_loop.join();

  return 0;
}
