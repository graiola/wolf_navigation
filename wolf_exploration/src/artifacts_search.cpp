#include "wolf_exploration/artifacts_search.h"

namespace wolf_exploration
{

ArtifactsSearch::ArtifactsSearch()
  : MoveBasePlanner()
  , converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
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

  std::string centroid_radius = "centroid_radius";
  private_nh_.param("centroid_radius", centroid_radius_,1.5); // [m]

  costmap_sub_ = private_nh_.subscribe(costmap_topic, 1, &ArtifactsSearch::costmapCallback, this);
  costmap_update_sub_ = private_nh_.subscribe(costmap_update_topic, 1, &ArtifactsSearch::costmapUpdateCallback, this);
  obstacle_pub_ = private_nh_.advertise<costmap_converter::ObstacleArrayMsg>(obstacles_topic, 1000);
  polygon_pub_ = private_nh_.advertise<visualization_msgs::Marker>(polygon_marker_topic, 1000);

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

  init();
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

  obstacles_ = *converter_->getObstacles();

  if (obstacles_.obstacles.empty())
    return;

  // publish
  if(visualize_)
    visualizePolygons();
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
  obstacles_ = *converter_->getObstacles();

  const std::vector<costmap_converter::ObstacleMsg>& obstacle_vector = obstacles_.obstacles;

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

  if (obstacles_.obstacles.empty())
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
        if(dist(*it_main,*it_comp) <= centroid_radius_)
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

  if (obstacles_.obstacles.empty())
    return;
}

bool ArtifactsSearch::makeGoal(const geometry_msgs::Pose &robot_pose, move_base_msgs::MoveBaseGoal &goal, double &goal_distance)
{
  // get closest centroid not on the black list
  double min_dist = 10000.0; // Dummy value
  double current_dist = 0.0;
  int min_idx = -1;
  for(unsigned int i=0; i < centroids_.size(); i++)
  {
    current_dist = dist(centroids_[i],robot_pose.position);
    if(current_dist <= min_dist)
    {
      min_dist = current_dist;
      min_idx = i;
    }
  }
  if(min_idx == -1)
    return false;

  geometry_msgs::Point target_position = centroids_[min_idx];

  // align the robot with the goal
  double yaw = 0.0;
  double xd, yd;
  yd = target_position.y-robot_pose.position.y;
  xd = target_position.x-robot_pose.position.x;
  yaw = std::atan2(yd,xd);
  tf2::Quaternion q;
  q.setRPY( 0., 0., yaw );
  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  double d = std::sqrt(xd*xd + yd*yd) - centroid_radius_; // FIXME
  goal.target_pose.pose.position.x = d * std::cos(yaw) + robot_pose.position.x;
  goal.target_pose.pose.position.y = d * std::sin(yaw) + robot_pose.position.y;

  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();

}

void ArtifactsSearch::visualizePolygons()
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id =  costmap_client_.getGlobalFrameID();
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "polygons";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color = green_;

  for (const costmap_converter::ObstacleMsg& obstacle : obstacles_.obstacles)
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

  obstacle_pub_.publish(obstacles_);
  polygon_pub_.publish(line_list);
}

}  // namespace
