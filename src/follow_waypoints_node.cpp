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

#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>

#include <rt_gui/rt_gui_client.h>

#include <deque>
#include <mutex>

#define NODE_NAME "follow_waypoints_node"

using namespace rt_gui;

bool _running = false;
bool _patrol_mode = false;
double _wait_duration = 360.0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::shared_ptr<MoveBaseClient> _move_base;

struct Waypoints
{
public:

  void addWaypoint(const move_base_msgs::MoveBaseGoal& waypoint)
  {

  }

  void addWaypoint(const geometry_msgs::PointStamped& waypoint)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    unsigned int id_waypoint = list_.size()+1;

    move_base_msgs::MoveBaseGoal goal;
    // fill in the navigation goal message
    goal.target_pose.header.frame_id = waypoint.header.frame_id;
    goal.target_pose.header.stamp    = waypoint.header.stamp;
    goal.target_pose.pose.position.x = waypoint.point.x;
    goal.target_pose.pose.position.y = waypoint.point.y;
    goal.target_pose.pose.position.z = 0.0;

    // FIXME align the robot with the heading of the current waypoint wrt the previous one
    double yaw = 0.0;

    if(list_.size()!=0)
      for(unsigned int i=0;i<list_.size();i++)
        if(list_[i].first == id_waypoint - 1)
          yaw = std::atan2(waypoint.point.y-list_[i].second.target_pose.pose.position.y,waypoint.point.x-list_[i].second.target_pose.pose.position.x);
    else
          yaw = std::atan2(waypoint.point.y,waypoint.point.x);

    tf2::Quaternion q;
    q.setRPY( 0., 0., yaw );
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();



    ROS_INFO_NAMED(NODE_NAME,"Add waypoint %i -> [x: %f, y: %f, z: %f, r: %f, p: %f, y: %f]",
                   id_waypoint,waypoint.point.x,waypoint.point.y,waypoint.point.z,0.0,0.0,yaw);

    list_.push_back(std::pair<unsigned int,move_base_msgs::MoveBaseGoal>(id_waypoint,goal));
  }

  move_base_msgs::MoveBaseGoal getCurrentWaypoint()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return list_[0].second; // FIXME
  }

  unsigned int getCurrentWaypointId()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return list_[0].first; // FIXME
  }

  void removeCurrentWaypoint()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    list_.pop_front();
  }

  void moveToNextWaypoint()
  {
    std::lock_guard<std::mutex> lk(mtx_);


    //std::cout << "BEFORE" << std::endl;
    //for(unsigned int i=0;i<list_.size();i++)
    //  std::cout << "w " << list_[i].first << std::endl;

    std::rotate(list_.begin(), list_.begin() + 1, list_.end());

    //std::cout << "AFTER" << std::endl;
    //for(unsigned int i=0;i<list_.size();i++)
    //  std::cout << "w " << list_[i].first << std::endl;

  }

  unsigned int getNumberOfWaypoints()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return list_.size();
  }

private:
  std::deque<std::pair<unsigned int,move_base_msgs::MoveBaseGoal> > list_;
  std::mutex mtx_;
} _waypoints;

void callback(const geometry_msgs::PointStamped& waypoint)
{
  _waypoints.addWaypoint(waypoint);
}

void loop()
{

  while(ros::ok())
  {
    if(_running && _waypoints.getNumberOfWaypoints())
    {
      auto goal    = _waypoints.getCurrentWaypoint();
      auto goal_id = _waypoints.getCurrentWaypointId();

      ROS_INFO_NAMED(NODE_NAME,"The robot is moving toward waypoint %i",goal_id);

      _move_base->sendGoal(goal);

      _move_base->waitForResult(ros::Duration(_wait_duration));
      if (_move_base->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        if(_patrol_mode)
          _waypoints.moveToNextWaypoint();
        else
          _waypoints.removeCurrentWaypoint();
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
