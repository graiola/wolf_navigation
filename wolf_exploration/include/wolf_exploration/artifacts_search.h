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
*/

#ifndef WOLF_EXPLORATION_ARTIFACTS_SEARCH_H
#define WOLF_EXPLORATION_ARTIFACTS_SEARCH_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.h>

#include "wolf_exploration/move_base_planner.h"

namespace wolf_exploration {

class ArtifactsSearch : public MoveBasePlanner
{

public:

  const std::string CLASS_NAME = "ArtifactsSearch";

  typedef std::shared_ptr<ArtifactsSearch> Ptr;

  ArtifactsSearch();

  void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

  void costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& update);

protected:

  virtual bool makeGoal(const geometry_msgs::Pose& robot_pose, move_base_msgs::MoveBaseGoal& goal, double& goal_distance);

  void visualizePolygons();

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader_;
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;
  std::vector<geometry_msgs::Point> centroids_;
  costmap_converter::ObstacleArrayMsg obstacles_;

  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher polygon_pub_;

  // parameters
  int occupied_min_value_;
  double centroid_radius_;

};

} // namespace

#endif

