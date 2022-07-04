#include "wolf_exploration/move_base_planner.h"

namespace wolf_exploration
{

MoveBasePlanner::MoveBasePlanner(std::string planner_name, std::string move_base_client_name)
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , prev_distance_(0)
  , planner_name_(planner_name)
  , move_base_client_(move_base_client_name)
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , running_(false)
{
  // Colors definition
  blue_.r = 0;
  blue_.g = 0;
  blue_.b = 1.0;
  red_.r = 1.0;
  red_.g = 0;
  red_.b = 0;
  green_.r = 0;
  green_.g = 1.0;
  green_.b = 0;
  blue_.a = green_.a = red_.a = 1.0;

  double timeout;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, true);
}

MoveBasePlanner::~MoveBasePlanner()
{
  stop();
}

void MoveBasePlanner::init()
{
  ROS_INFO_NAMED(CLASS_NAME,"%s waiting to connect to move_base server",planner_name_.c_str());
  move_base_client_.waitForServer();
  ROS_INFO_NAMED(CLASS_NAME,"%s connected to move_base server",planner_name_.c_str());
  ROS_INFO_NAMED(CLASS_NAME,"%s in stand-by",planner_name_.c_str());
}

void MoveBasePlanner::start()
{
  running_ = true;
  ROS_INFO_NAMED(CLASS_NAME,"%s started",planner_name_.c_str());
}

void MoveBasePlanner::stop()
{
  running_ = false;
  move_base_client_.cancelAllGoals();
  goals_blacklist_.clear();
  ROS_INFO_NAMED(CLASS_NAME,"%s stopped",planner_name_.c_str());
}

void MoveBasePlanner::pause()
{
  running_ = false;
  move_base_client_.cancelAllGoals();
  ROS_INFO_NAMED(CLASS_NAME,"%s paused",planner_name_.c_str());
}

void MoveBasePlanner::makePlan()
{
  while(ros::ok())
  {
    if(running_ && mtx_.try_lock())
    {

      if(visualize_)
        visualizeGoals();

      auto robot_pose = costmap_client_.getRobotPose();

      move_base_msgs::MoveBaseGoal goal;
      double goal_distance;

      if(!makeGoal(robot_pose,goal,goal_distance) && !goalOnBlacklist(goal))
      {
        ROS_INFO_NAMED(CLASS_NAME,"%s can not create a new goal",planner_name_.c_str());
        stop();
        continue;
      }

      // time out if we are not making any progress
      bool same_goal = prev_goal_ == goal;
      prev_goal_ = goal;
      if (!same_goal || prev_distance_ > goal_distance) {
        // we have different goal or we made some progress
        last_progress_ = ros::Time::now();
        prev_distance_ = goal_distance;
      }
      // black list if we've made no progress for a long time
      if (ros::Time::now() - last_progress_ > progress_timeout_)
      {
        goals_blacklist_.push_back(goal);
        ROS_INFO_NAMED(CLASS_NAME,"%s adding current goal to black list",planner_name_.c_str());
        continue;
      }
      // we don't need to do anything if we still pursuing the same goal
      if (same_goal)
        continue;

      goals_.push_back(goal);

      move_base_client_.sendGoal(goal);
      move_base_client_.waitForResult(progress_timeout_);
      auto status = move_base_client_.getState();

      ROS_INFO_NAMED(CLASS_NAME,"%s reached goal with status: %s", planner_name_.c_str(), status.toString().c_str());
      if (status == actionlib::SimpleClientGoalState::ABORTED || status == actionlib::SimpleClientGoalState::SUCCEEDED)
        goals_blacklist_.push_back(goal);

      ROS_INFO_STREAM_NAMED(CLASS_NAME,"Send new goal at (" << goal.target_pose.pose.position.x << ", " << goal.target_pose.pose.position.y << ", " << goal.target_pose.pose.position.z <<")" );

      mtx_.unlock();
    } // running_

    ros::Duration(1. / planner_frequency_).sleep();
  } // while(ros::ok())
}

bool MoveBasePlanner::goalOnBlacklist(const move_base_msgs::MoveBaseGoal& goal, const double& radius)
{
  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& current_goal : goals_blacklist_)
  {
    if (dist(current_goal,goal) <= radius)
      return true;
  }
  return false;
}

bool MoveBasePlanner::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& current_goal : goals_blacklist_) {
    double x_diff = fabs(goal.x - current_goal.target_pose.pose.position.x);
    double y_diff = fabs(goal.y - current_goal.target_pose.pose.position.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

bool MoveBasePlanner::goalOnBlacklist(const move_base_msgs::MoveBaseGoal& goal)
{
  return goalOnBlacklist(goal.target_pose.pose.position);
}

void MoveBasePlanner::visualizeGoals()
{
  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = costmap_client_.getGlobalFrameID();
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "goals";
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w = 1.0;

  sphere_list.id = 0;
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

  sphere_list.scale.x = sphere_list.scale.y = sphere_list.scale.z = 0.3;

  for (std::size_t i=0; i<goals_.size(); ++i)
  {
    if(goalOnBlacklist(goals_[i]))
      sphere_list.colors.push_back(red_);
    else
      sphere_list.colors.push_back(green_);
    sphere_list.points.push_back(goals_[i].target_pose.pose.position);
  }

  goals_pub_.publish(sphere_list);
}

}  // namespace
