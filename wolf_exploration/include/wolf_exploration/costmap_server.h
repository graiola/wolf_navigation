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

#ifndef WOLF_EXPLORATION_COSTMAP_SERVER_H
#define WOLF_EXPLORATION_COSTMAP_SERVER_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace wolf_exploration
{
class Costmap2DServer
{
public:

  Costmap2DServer(const std::string &costmap_name);

  void start();
  void stop();
  void pause();

protected:

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;


};

}  // namespace explore

#endif
