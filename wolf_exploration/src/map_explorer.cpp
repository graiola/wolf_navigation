#include "wolf_exploration/map_explorer.h"

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace wolf_exploration
{

MapExplorer::MapExplorer()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
  , running_(false)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO_NAMED(CLASS_NAME,"Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO_NAMED(CLASS_NAME,"Connected to move_base server");

  //exploring_timer_ =
  //    relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
  //                             [this](const ros::TimerEvent&) { makePlan(); },false,false);

  ROS_INFO_NAMED(CLASS_NAME,"Map exploration in stand-by");
}

MapExplorer::~MapExplorer()
{
  stop();
}

void MapExplorer::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG_NAMED(CLASS_NAME,"Visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void MapExplorer::makePlan()
{
  while(ros::ok())
  {
    if(running_)
    {
      // find frontiers
      auto pose = costmap_client_.getRobotPose();
      // get frontiers sorted according to cost
      auto frontiers = search_.searchFrom(pose.position);
      ROS_DEBUG_NAMED(CLASS_NAME,"Found %lu frontiers", frontiers.size());
      for (size_t i = 0; i < frontiers.size(); ++i) {
        ROS_DEBUG_NAMED(CLASS_NAME,"Frontier %zd cost: %f", i, frontiers[i].cost);
      }

      if (frontiers.empty()) {
        ROS_INFO_NAMED(CLASS_NAME,"No more frontiers left to explore");
        stop();
        continue;
      }

      // publish frontiers as visualization markers
      if (visualize_) {
        visualizeFrontiers(frontiers);
      }

      // find non blacklisted frontier
      auto frontier =
          std::find_if_not(frontiers.begin(), frontiers.end(),
                           [this](const frontier_exploration::Frontier& f) {
        return goalOnBlacklist(f.centroid);
      });
      if (frontier == frontiers.end()) {
        stop();
        continue;
      }
      geometry_msgs::Point target_position = frontier->centroid;

      // time out if we are not making any progress
      bool same_goal = prev_goal_ == target_position;
      prev_goal_ = target_position;
      if (!same_goal || prev_distance_ > frontier->min_distance) {
        // we have different goal or we made some progress
        last_progress_ = ros::Time::now();
        prev_distance_ = frontier->min_distance;
      }
      // black list if we've made no progress for a long time
      if (ros::Time::now() - last_progress_ > progress_timeout_) {
        frontier_blacklist_.push_back(target_position);
        ROS_INFO_NAMED(CLASS_NAME,"Adding current goal to black list");
        makePlan();
        continue;
      }

      // we don't need to do anything if we still pursuing the same goal
      if (same_goal) {
        continue;
      }

      // send goal to move_base if we have something new to pursue
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.pose.position = target_position;
      goal.target_pose.pose.orientation.w = 1.;
      goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
      goal.target_pose.header.stamp = ros::Time::now();
      move_base_client_.sendGoal(
            goal, [this, target_position](
            const actionlib::SimpleClientGoalState& status,
            const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
      ROS_INFO_STREAM_NAMED(CLASS_NAME,"Send exploration goal at (" << target_position.x << ", " << target_position.y << ", " << target_position.z <<")" );
    } // running_

    ros::Duration(1. / planner_frequency_).sleep();
  } // while(ros::ok())
}

bool MapExplorer::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void MapExplorer::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG_NAMED(CLASS_NAME,"Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG_NAMED(CLASS_NAME,"Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  //oneshot_ = relative_nh_.createTimer(
  //    ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //    true);
}

void MapExplorer::start()
{
  running_ = true;
  ROS_INFO_NAMED(CLASS_NAME,"Map exploration started");
}

void MapExplorer::stop()
{
  running_ = false;
  move_base_client_.cancelAllGoals();
  ROS_INFO_NAMED(CLASS_NAME,"Map exploration stopped");
}

}  // namespace