#ifndef WOLF_EXPLORATION_ARTIFACTS_SEARCH_H
#define WOLF_EXPLORATION_ARTIFACTS_SEARCH_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "wolf_exploration/costmap_client.h"

namespace wolf_exploration {

class ArtifactsSearch
{

public:

  const std::string CLASS_NAME = "ArtifactsSearch";

  ArtifactsSearch();
  ~ArtifactsSearch();

  /**
   * @brief  Start search
   */
  void start();

  /**
   * @brief  Stop search
   */
  void stop();

  /**
   * @brief Create a plan
   */
  void makePlan();

  void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

  void costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& update);

private:

  bool goalOnBlacklist(const geometry_msgs::Point& goal);

  void publishAsMarker(const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub);
  void publishAsMarker(const std::vector<geometry_msgs::Point>& points, ros::Publisher& marker_pub);
  void publishAsMarker(const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub);

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader_;
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher polygon_pub_;
  ros::Publisher centroid_pub_;
  tf::TransformListener tf_listener_;

  std::vector<geometry_msgs::Point> centroids_;
  std::vector<geometry_msgs::Point> centroid_blacklist_;
  geometry_msgs::Point prev_goal_;
  double prev_distance_;
  ros::Time last_progress_;
  ros::Duration progress_timeout_;
  size_t last_markers_count_;

  // parameters
  double planner_frequency_;
  bool visualize_;
  int occupied_min_value_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

  Costmap2DClient costmap_client_;

  double centroid_radius_;

  std::atomic<bool> running_;

  std::mutex mtx_;

};

} // namespace

#endif

