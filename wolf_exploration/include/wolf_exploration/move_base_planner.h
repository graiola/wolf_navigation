#ifndef WOLF_EXPLORATION_MOVE_BASE_PLANNER_H
#define WOLF_EXPLORATION_MOVE_BASE_PLANNER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>

#include "wolf_exploration/costmap_client.h"

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

