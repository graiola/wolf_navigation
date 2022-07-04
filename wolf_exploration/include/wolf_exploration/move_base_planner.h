/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
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
 *
*/

#ifndef WOLF_EXPLORATION_MOVE_BASE_PLANNER_H
#define WOLF_EXPLORATION_MOVE_BASE_PLANNER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include "wolf_exploration/costmap_client.h"

#define TOL 0.01

inline static double dist(const geometry_msgs::Point& one,
                          const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist;
}

inline static double dist(const move_base_msgs::MoveBaseGoal& one,
                          const move_base_msgs::MoveBaseGoal& two)
{
  return dist(one.target_pose.pose.position,two.target_pose.pose.position);
}

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < TOL;
}

inline static bool operator==(const move_base_msgs::MoveBaseGoal& one,
                              const move_base_msgs::MoveBaseGoal& two)
{
  return (one.target_pose.pose.position == two.target_pose.pose.position);
}

namespace wolf_exploration {

class MoveBasePlanner
{

public:

  const std::string CLASS_NAME = "MoveBasePlanner";

  typedef std::shared_ptr<MoveBasePlanner> Ptr;

  MoveBasePlanner();
  ~MoveBasePlanner();

  /**
   * @brief  Start the planner
   */
  void start();

  /**
   * @brief  Stop the planner
   */
  void stop();

  /**
   * @brief  Pause the planner
   */
  void pause();

  /**
   * @brief Create a plan for move_base
   */
  void makePlan();

protected:

  void init();

  virtual bool makeGoal(const geometry_msgs::Pose& robot_pose, move_base_msgs::MoveBaseGoal& goal, double& goal_distance) = 0;

  bool goalOnBlacklist(const move_base_msgs::MoveBaseGoal& goal, const double &radius);

  bool goalOnBlacklist(const move_base_msgs::MoveBaseGoal& goal);

  bool goalOnBlacklist(const geometry_msgs::Point& point);

  void visualizeGoals();

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  tf::TransformListener tf_listener_;

  std::vector<move_base_msgs::MoveBaseGoal> goals_;
  std::vector<move_base_msgs::MoveBaseGoal> goals_blacklist_;
  move_base_msgs::MoveBaseGoal prev_goal_;
  double prev_distance_;
  ros::Time last_progress_;

  // parameters
  double planner_frequency_;
  bool visualize_;
  ros::Duration progress_timeout_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

  ros::Publisher goals_pub_;

  std_msgs::ColorRGBA blue_; // In visited list
  std_msgs::ColorRGBA red_; // In black list
  std_msgs::ColorRGBA green_; // To visit

  Costmap2DClient costmap_client_;

  std::atomic<bool> running_;

  std::mutex mtx_;

};

} // namespace

#endif

