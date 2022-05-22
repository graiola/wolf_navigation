#ifndef COSTMAP_SERVER_H
#define COSTMAP_SERVER_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace explore
{
class Costmap2DServer
{
public:

  Costmap2DServer(const std::string &costmap_name = "artifacts_costmap");

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
