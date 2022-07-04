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

#ifndef WOLF_EXPLORATION_COSTMAP_CLIENT_H
#define WOLF_EXPLORATION_COSTMAP_CLIENT_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace wolf_exploration
{
class Costmap2DClient
{
public:
  /**
   * @brief Contructs client and start listening
   * @details Constructor will block until first map update is received and
   * map is ready to use, also will block before trasformation
   * robot_base_frame <-> global_frame is available.
   *
   * @param param_nh node hadle to retrieve parameters from
   * @param subscription_nh node hadle where topics will be subscribed
   * @param tf_listener Will be used for transformation of robot pose.
   */
  Costmap2DClient(ros::NodeHandle& param_nh, ros::NodeHandle& subscription_nh,
                  const tf::TransformListener* tf_listener);
  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @return pose of the robot in the global frame of the costmap
   */
  geometry_msgs::Pose getRobotPose() const;

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from
   * all the layers.
   *
   * This pointer will stay the same for the lifetime of Costmap2DClient object.
   */
  costmap_2d::Costmap2D* getCostmap()
  {
    return &costmap_;
  }

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from
   * all the layers.
   *
   * This pointer will stay the same for the lifetime of Costmap2DClient object.
   */
  const costmap_2d::Costmap2D* getCostmap() const
  {
    return &costmap_;
  }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  const std::string& getGlobalFrameID() const
  {
    return global_frame_;
  }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  const std::string& getBaseFrameID() const
  {
    return robot_base_frame_;
  }

protected:
  void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);

  costmap_2d::Costmap2D costmap_;

  const tf::TransformListener* const tf_;  ///< @brief Used for transforming
                                           /// point clouds
  std::string global_frame_;      ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;    ///< timeout before transform errors

private:
  // will be unsubscribed at destruction
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_updates_sub_;
};

}  // namespace explore

#endif
