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
 * original code and license notice here: http://wiki.ros.org/explore_lite
*/

#ifndef WOLF_EXPLORATION_MAP_EXPLORER_H
#define WOLF_EXPLORATION_MAP_EXPLORER_H

#include "wolf_exploration/costmap_client.h"
#include "wolf_exploration/frontier_search.h"
#include "wolf_exploration/move_base_planner.h"

namespace wolf_exploration
{
/**
 * @class MapExplorer
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class MapExplorer : public MoveBasePlanner
{

public:

  const std::string CLASS_NAME = "MapExplorer";

  typedef std::shared_ptr<MapExplorer> Ptr;

  MapExplorer();

protected:

   virtual bool makeGoal(const geometry_msgs::Pose& robot_pose, move_base_msgs::MoveBaseGoal& goal, double& goal_distance);

  /**
   * @brief Publish a frontiers as markers
   */
  void visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers);

  ros::Publisher marker_array_publisher_;

  frontier_exploration::FrontierSearch search_;

  size_t last_markers_count_;

  // parameters
  double potential_scale_;
  double orientation_scale_;
  double gain_scale_;

};

} // namespace

#endif
