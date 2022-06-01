#ifndef WOLF_EXPLORATION_ARTIFACTS_SEARCH_H
#define WOLF_EXPLORATION_ARTIFACTS_SEARCH_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.h>

namespace wolf_exploration {

class ArtifactsSearch
{
public:

  ArtifactsSearch();

  void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

  void costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& update);

  void publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub);

  void publishAsMarker(const std::string& frame_id, const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub);

  void publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::Point>& points, ros::Publisher& marker_pub);

private:
  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader_;
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;

  ros::NodeHandle n_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher polygon_pub_;
  ros::Publisher centroid_pub_;

  std::string frame_id_;
  int occupied_min_value_;

  costmap_2d::Costmap2D map_;

};

} // namespace

#endif

