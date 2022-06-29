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
  ~ArtifactsSearch();

  void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

  void costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& update);

protected:

  virtual bool makeGoal(const geometry_msgs::Pose& robot_pose, move_base_msgs::MoveBaseGoal& goal, double& goal_distance);

  void publishAsMarker(const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub);
  void publishAsMarker(const std::vector<geometry_msgs::Point>& points, ros::Publisher& marker_pub);
  void publishAsMarker(const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub);

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader_;
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;

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

