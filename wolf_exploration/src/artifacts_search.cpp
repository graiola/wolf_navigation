#include "wolf_exploration/artifacts_search.h"

using namespace wolf_exploration;

ArtifactsSearch::ArtifactsSearch() : converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"), n_("~")
{
  // load converter plugin from parameter server, otherwise set default
  std::string converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
  n_.param("converter_plugin", converter_plugin, converter_plugin);

  try
  {
    converter_ = converter_loader_.createInstance(converter_plugin);
  }
  catch(const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    ros::shutdown();
  }

  ROS_INFO_STREAM("Standalone costmap converter:" << converter_plugin << " loaded.");

  std::string costmap_topic = "/artifacts_search/artifacts_costmap/costmap";
  n_.param("costmap_topic", costmap_topic, costmap_topic);

  std::string costmap_update_topic = "/artifacts_search/artifacts_costmap/costmap_updates";
  n_.param("costmap_update_topic", costmap_update_topic, costmap_update_topic);

  std::string obstacles_topic = "costmap_obstacles";
  n_.param("obstacles_topic", obstacles_topic, obstacles_topic);

  std::string polygon_marker_topic = "costmap_polygon_markers";
  n_.param("polygon_marker_topic", polygon_marker_topic, polygon_marker_topic);

  std::string centroid_marker_topic = "costmap_centroid_markers";
  n_.param("centroid_marker_topic", centroid_marker_topic, centroid_marker_topic);

  costmap_sub_ = n_.subscribe(costmap_topic, 1, &ArtifactsSearch::costmapCallback, this);
  costmap_update_sub_ = n_.subscribe(costmap_update_topic, 1, &ArtifactsSearch::costmapUpdateCallback, this);
  obstacle_pub_ = n_.advertise<costmap_converter::ObstacleArrayMsg>(obstacles_topic, 1000);
  polygon_pub_ = n_.advertise<visualization_msgs::Marker>(polygon_marker_topic, 10);
  centroid_pub_ = n_.advertise<visualization_msgs::Marker>(centroid_marker_topic, 10);

  occupied_min_value_ = 100;
  n_.param("occupied_min_value", occupied_min_value_, occupied_min_value_);

  std::string odom_topic = "/odom";
  n_.param("odom_topic", odom_topic, odom_topic);

  if (converter_)
  {
    converter_->setOdomTopic(odom_topic);
    converter_->initialize(n_);
    converter_->setCostmap2D(&map_);
    //converter_->startWorker(ros::Rate(5), &map, true);
  }
}


void ArtifactsSearch::costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  ROS_INFO_ONCE("Got first costmap callback. This message will be printed once");

  if (msg->info.width != map_.getSizeInCellsX() || msg->info.height != map_.getSizeInCellsY() || msg->info.resolution != map_.getResolution())
  {
    ROS_INFO("New map format, resizing and resetting map...");
    map_.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
  }
  else
  {
    map_.updateOrigin(msg->info.origin.position.x, msg->info.origin.position.y);
  }


  for (std::size_t i=0; i < msg->data.size(); ++i)
  {
    unsigned int mx, my;
    map_.indexToCells((unsigned int)i, mx, my);
    map_.setCost(mx, my, msg->data[i] >= occupied_min_value_ ? 255 : 0 );
  }

  // convert
  converter_->updateCostmap2D();
  converter_->compute();
  costmap_converter::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

  if (!obstacles)
    return;

  obstacle_pub_.publish(obstacles);

  frame_id_ = msg->header.frame_id;

  publishAsMarker(frame_id_, *obstacles, polygon_pub_, Color(0.0,1.0,0.0));
}

void ArtifactsSearch::costmapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; ++y)
  {
    for (unsigned int x = 0; x < update->width ; ++x)
    {
      map_.setCost(x, y, update->data[di++] >= occupied_min_value_ ? 255 : 0 );
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

  obstacle_pub_.publish(obstacles);

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
        double x = (it_main->x - it_comp->x);
        double y = (it_main->y - it_comp->y);
        if(std::sqrt( x*x + y*y ) <= 1.5)
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


  // Publish
  publishAsMarker(frame_id_, *obstacles, polygon_pub_, Color(0.0,1.0,0.0));
  publishAsMarker(frame_id_, centroids, centroid_pub_, Color(0.0,0.0,1.0));
  //publishAsMarker(frame_id_, centroids_original, centroid_original_pub_, Color(0.0,0.0,1.0));
}

void ArtifactsSearch::publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::Point>& points, ros::Publisher& marker_pub, Color color)
{
  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = frame_id;
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "Centroids";
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w = 1.0;

  sphere_list.id = 0;
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

  sphere_list.scale.x = sphere_list.scale.y = sphere_list.scale.z = 0.3;
  sphere_list.color.r = color.r_;
  sphere_list.color.g = color.g_;
  sphere_list.color.b = color.b_;
  sphere_list.color.a = 1.0;

  for (std::size_t i=0; i<points.size(); ++i)
    sphere_list.points.push_back(points[i]);

  marker_pub.publish(sphere_list);
}


void ArtifactsSearch::publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub, Color color)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "Polygons";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color.r = color.r_;
  line_list.color.g = color.g_;
  line_list.color.b = color.b_;
  line_list.color.a = 1.0;

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

void ArtifactsSearch::publishAsMarker(const std::string& frame_id, const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub, Color color)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "Polygons";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color.r = color.r_;
  line_list.color.g = color.g_;
  line_list.color.b = color.b_;
  line_list.color.a = 1.0;

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
