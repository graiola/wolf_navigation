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

#include <rt_gui/rt_gui_client.h>

#include "wolf_navigation/waypoints.h"

using namespace rt_gui;

static bool _running = false;
static bool _patrol_mode = false;
static double _wait_duration = 360.0;
static std::shared_ptr<MoveBaseClient> _move_base;
static Waypoints::Ptr _waypoints;

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
      auto goal    = _waypoints->getCurrentWaypointGoal();
      auto id      = _waypoints->getCurrentWaypointId();

      if(id != -1)
      {
        ROS_INFO_NAMED(NODE_NAME,"The robot is moving toward waypoint %i",id);

        _move_base->sendGoal(goal);

        _move_base->waitForResult(ros::Duration(_wait_duration));
        auto state = _move_base->getState();

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          if(_patrol_mode)
            _waypoints->moveToNextWaypoint();
          else
            _waypoints->removeWaypoint(id);
          ROS_INFO_NAMED(NODE_NAME,"The robot reached waypoint %i",id);
        }
        else if(state == actionlib::SimpleClientGoalState::ABORTED)
        {
          ROS_WARN_NAMED(NODE_NAME,"The robot did not reach waypoint %i, aborted",id);
        }
        else
          ROS_WARN_NAMED(NODE_NAME,"The robot failed to reach waypoint %i",id);
      }
      else
        ROS_WARN_NAMED(NODE_NAME,"Invalid waypoint id %i",id);

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

  _waypoints.reset(new Waypoints(n,_move_base));

  ros::Subscriber sub = n.subscribe("waypoints", 1000, callback);

  // create interface
  if(RtGuiClient::getIstance().init("wolf_panel","follow_waypoints"))
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
