#include "wolf_exploration/artifacts_search.h"

inline static double dist(const geometry_msgs::Point& one,
                          const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist;
}

namespace wolf_exploration
{

ArtifactsSearch::ArtifactsSearch()
  : converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
  , private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , prev_distance_(0)
  , last_markers_count_(0)
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , running_(false)
{
  // load converter plugin from parameter server, otherwise set default
  std::string converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
  relative_nh_.param("converter_plugin", converter_plugin, converter_plugin);

  try
  {
    converter_ = converter_loader_.createInstance(converter_plugin);
  }
  catch(const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_NAMED(CLASS_NAME,"The plugin failed to load for some reason. Error: %s", ex.what());
    ros::shutdown();
  }

  ROS_INFO_STREAM_NAMED(CLASS_NAME,"Standalone costmap converter:" << converter_plugin << " loaded.");

  double timeout;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, true);

  std::string costmap_topic = "/artifacts_search/artifacts_costmap/costmap";
  private_nh_.param("costmap_topic", costmap_topic, costmap_topic);

  std::string costmap_update_topic = "/artifacts_search/artifacts_costmap/costmap_updates";
  private_nh_.param("costmap_update_topic", costmap_update_topic, costmap_update_topic);

  std::string obstacles_topic = "costmap_obstacles";
  private_nh_.param("obstacles_topic", obstacles_topic, obstacles_topic);

  std::string polygon_marker_topic = "costmap_polygon_markers";
  private_nh_.param("polygon_marker_topic", polygon_marker_topic, polygon_marker_topic);

  std::string centroid_marker_topic = "costmap_centroid_markers";
  private_nh_.param("centroid_marker_topic", centroid_marker_topic, centroid_marker_topic);

  costmap_sub_ = private_nh_.subscribe(costmap_topic, 1, &ArtifactsSearch::costmapCallback, this);
  costmap_update_sub_ = private_nh_.subscribe(costmap_update_topic, 1, &ArtifactsSearch::costmapUpdateCallback, this);
  obstacle_pub_ = private_nh_.advertise<costmap_converter::ObstacleArrayMsg>(obstacles_topic, 1000);
  polygon_pub_ = private_nh_.advertise<visualization_msgs::Marker>(polygon_marker_topic, 10);
  centroid_pub_ = private_nh_.advertise<visualization_msgs::Marker>(centroid_marker_topic, 10);

  occupied_min_value_ = 100;
  private_nh_.param("occupied_min_value", occupied_min_value_, occupied_min_value_);

  std::string odom_topic = "/odom";
  private_nh_.param("odom_topic", odom_topic, odom_topic);

  if (converter_)
  {
    converter_->setOdomTopic(odom_topic);
    converter_->initialize(private_nh_);
    converter_->setCostmap2D(costmap_client_.getCostmap());
    //converter_->startWorker(ros::Rate(5), &map, true);
  }

  ROS_INFO_NAMED(CLASS_NAME,"Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO_NAMED(CLASS_NAME,"Connected to move_base server");

  ROS_INFO_NAMED(CLASS_NAME,"Artifacts search in stand-by");
}

ArtifactsSearch::~ArtifactsSearch()
{
  stop();
}

void ArtifactsSearch::start()
{
  running_ = true;
  ROS_INFO_NAMED(CLASS_NAME,"Artifacts search started");
}

void ArtifactsSearch::stop()
{
  running_ = false;
  move_base_client_.cancelAllGoals();
  ROS_INFO_NAMED(CLASS_NAME,"Artifacts search stopped");
}

void ArtifactsSearch::makePlan()
{
  while(ros::ok())
  {
    if(running_ && mtx_.try_lock())
    {

      if (centroids_.empty()) {
        ROS_INFO_NAMED(CLASS_NAME,"No more occupied spaces left to explore");
        stop();
        continue;
      }

      auto pose = costmap_client_.getRobotPose();
      // get closest centroid
      double min_dist = 10000.0; // Dummy value
      double current_dist = 0.0;
      unsigned int min_idx = 0;
      for(unsigned int i=0; i < centroids_.size(); i++)
      {
        current_dist = dist(centroids_[i],pose.position);
        if(current_dist <= min_dist)
        {
          min_dist = current_dist;
          min_idx = i;
        }
      }

      // find non blacklisted centroids
      auto centroid =
          std::find_if_not(centroids_.begin(), centroids_.end(),
                           [this](const geometry_msgs::Point& c) {
        return goalOnBlacklist(c);
      });
      if (centroid == centroids_.end()) {
        stop();
        continue;
      }
      geometry_msgs::Point target_position = *centroid;

      // time out if we are not making any progress
      bool same_goal = prev_goal_ == target_position;
      prev_goal_ = target_position;
      if (!same_goal || prev_distance_ > min_dist) {
        // we have different goal or we made some progress
        last_progress_ = ros::Time::now();
        prev_distance_ = min_dist;
      }
      // black list if we've made no progress for a long time
      if (ros::Time::now() - last_progress_ > progress_timeout_) {
        centroid_blacklist_.push_back(target_position);
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

    mtx_.unlock();
    } // running_

    ros::Duration(1. / planner_frequency_).sleep();
  } // while(ros::ok())
}

void ArtifactsSearch::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& centroid_goal)
{
  ROS_DEBUG_NAMED(CLASS_NAME,"Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    centroid_blacklist_.push_back(centroid_goal);
    ROS_DEBUG_NAMED(CLASS_NAME,"Adding current goal to black list");
  }
}


void ArtifactsSearch::costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  ROS_INFO_ONCE_NAMED(CLASS_NAME,"Got first costmap callback. This message will be printed once");

  mtx_.lock();

  if (msg->info.width != costmap_client_.getCostmap()->getSizeInCellsX() || msg->info.height != costmap_client_.getCostmap()->getSizeInCellsY() || msg->info.resolution != costmap_client_.getCostmap()->getResolution())
  {
    ROS_INFO("New map format, resizing and resetting map...");
    costmap_client_.getCostmap()->resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
  }
  else
  {
    costmap_client_.getCostmap()->updateOrigin(msg->info.origin.position.x, msg->info.origin.position.y);
  }

  for (std::size_t i=0; i < msg->data.size(); ++i)
  {
    unsigned int mx, my;
    costmap_client_.getCostmap()->indexToCells((unsigned int)i, mx, my);
    costmap_client_.getCostmap()->setCost(mx, my, msg->data[i] >= occupied_min_value_ ? 255 : 0 );
  }

  // convert
  converter_->updateCostmap2D();
  converter_->compute();

  mtx_.unlock();

  costmap_converter::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

  if (!obstacles)
    return;

  // publish
  if(visualize_)
  {
    obstacle_pub_.publish(obstacles);
    publishAsMarker(*obstacles, polygon_pub_);
  }
}

void ArtifactsSearch::costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;

  mtx_.lock();

  for (unsigned int y = 0; y < update->height ; ++y)
  {
    for (unsigned int x = 0; x < update->width ; ++x)
    {
      costmap_client_.getCostmap()->setCost(x, y, update->data[di++] >= occupied_min_value_ ? 255 : 0 );
    }
  }

  // convert
  // TODO(roesmann): currently, the converter updates the complete costmap and not the part which is updated in this callback
  converter_->updateCostmap2D();
  converter_->compute();
  costmap_converter::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

  const std::vector<costmap_converter::ObstacleMsg>& obstacle_vector = obstacles->obstacles;

  std::vector<geometry_msgs::Point> centroids;

  double avg_x, avg_y, avg_z;
  geometry_msgs::Point centroid;
  for(unsigned int i=0; i< obstacle_vector.size(); i++)
  {
    avg_x = avg_y = avg_z = 0.0;
    unsigned int n_points = obstacle_vector[i].polygon.points.size();

    for(unsigned int j=0; j< n_points; j++)
    {
      avg_x = obstacle_vector[i].polygon.points[j].x + avg_x;
      avg_y = obstacle_vector[i].polygon.points[j].y + avg_y;
      avg_z = obstacle_vector[i].polygon.points[j].z + avg_z;
    }
    centroid.x = avg_x/n_points;
    centroid.y = avg_y/n_points;
    centroid.z = avg_z/n_points;
    centroids.push_back(centroid);
  }

  if (!obstacles)
    return;

  // Merge the centroids
  if(centroids.size() > 2)
  {
    std::vector<geometry_msgs::Point>::iterator it_main, it_comp;
    geometry_msgs::Point tmp_point;
    it_main = centroids.begin();
    it_comp = centroids.begin()+1;
    while (it_main != centroids.end())
    {
      it_comp = it_main+1;
      while (it_comp != centroids.end())
      {
        //double x = (it_main->x - it_comp->x);
        //double y = (it_main->y - it_comp->y);
        //if(std::sqrt( x*x + y*y ) <= 1.5)
        if(dist(*it_main,*it_comp) <= 1.5) // FIXME
        {
          tmp_point.x = (it_main->x + it_comp->x)/2.0;
          tmp_point.y = (it_main->y + it_comp->y)/2.0;
          *it_main = tmp_point;
          it_comp = centroids.erase(it_comp);
        }
        else {
          ++it_comp;
        }
      }
      ++it_main;
    }
  }

  centroids_ = centroids;

  mtx_.unlock();

  // Publish
  if(visualize_)
  {
    obstacle_pub_.publish(obstacles);
    publishAsMarker(*obstacles, polygon_pub_);
    publishAsMarker(centroids, centroid_pub_);
    //publishAsMarker(frame_id_, centroids_original, centroid_original_pub_, Color(0.0,0.0,1.0));
  }
}

bool ArtifactsSearch::goalOnBlacklist(const geometry_msgs::Point &goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& centroids : centroid_blacklist_) {
    double x_diff = fabs(goal.x - centroids.x);
    double y_diff = fabs(goal.y - centroids.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void ArtifactsSearch::publishAsMarker(const std::vector<geometry_msgs::Point>& points, ros::Publisher& marker_pub)
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

  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "centroids";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.orientation.w = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& p : points)
  {
    m.id = int(id);
    m.pose.position = p;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    if (goalOnBlacklist(p))
      m.color = red;
    else
      m.color = blue;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
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
  marker_pub.publish(markers_msg);
}


void ArtifactsSearch::publishAsMarker(const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub)
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

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = costmap_client_.getGlobalFrameID();
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "polygons";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color = green;

  for (std::size_t i=0; i<polygonStamped.size(); ++i)
  {
    for (int j=0; j< (int)polygonStamped[i].polygon.points.size()-1; ++j)
    {
      geometry_msgs::Point line_start;
      line_start.x = polygonStamped[i].polygon.points[j].x;
      line_start.y = polygonStamped[i].polygon.points[j].y;
      line_list.points.push_back(line_start);
      geometry_msgs::Point line_end;
      line_end.x = polygonStamped[i].polygon.points[j+1].x;
      line_end.y = polygonStamped[i].polygon.points[j+1].y;
      line_list.points.push_back(line_end);
    }
    // close loop for current polygon
    if (!polygonStamped[i].polygon.points.empty() && polygonStamped[i].polygon.points.size() != 2 )
    {
      geometry_msgs::Point line_start;
      line_start.x = polygonStamped[i].polygon.points.back().x;
      line_start.y = polygonStamped[i].polygon.points.back().y;
      line_list.points.push_back(line_start);
      if (line_list.points.size() % 2 != 0)
      {
        geometry_msgs::Point line_end;
        line_end.x = polygonStamped[i].polygon.points.front().x;
        line_end.y = polygonStamped[i].polygon.points.front().y;
        line_list.points.push_back(line_end);
      }
    }
  }

  marker_pub.publish(line_list);
}

void ArtifactsSearch::publishAsMarker(const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub)
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

  visualization_msgs::Marker line_list;
  line_list.header.frame_id =  costmap_client_.getGlobalFrameID();
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "polygons";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color = green;

  for (const costmap_converter::ObstacleMsg& obstacle : obstacles.obstacles)
  {
    for (int j=0; j< (int)obstacle.polygon.points.size()-1; ++j)
    {
      geometry_msgs::Point line_start;
      line_start.x = obstacle.polygon.points[j].x;
      line_start.y = obstacle.polygon.points[j].y;
      line_list.points.push_back(line_start);
      geometry_msgs::Point line_end;
      line_end.x = obstacle.polygon.points[j+1].x;
      line_end.y = obstacle.polygon.points[j+1].y;
      line_list.points.push_back(line_end);
    }
    // close loop for current polygon
    if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2 )
    {
      geometry_msgs::Point line_start;
      line_start.x = obstacle.polygon.points.back().x;
      line_start.y = obstacle.polygon.points.back().y;
      line_list.points.push_back(line_start);
      if (line_list.points.size() % 2 != 0)
      {
        geometry_msgs::Point line_end;
        line_end.x = obstacle.polygon.points.front().x;
        line_end.y = obstacle.polygon.points.front().y;
        line_list.points.push_back(line_end);
      }
    }
  }

  marker_pub.publish(line_list);
}

}  // namespace
