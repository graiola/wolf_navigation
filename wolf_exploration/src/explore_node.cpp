/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola, Federico Rollo
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include "wolf_exploration/explore.h"
#include "wolf_exploration/costmap_server.h"

#include <thread>
#include <rt_gui/rt_gui_client.h>
#include <ros/ros.h>
#include <cstdio>
#include <ros/console.h>
#include <nav_msgs/GetMap.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#define NODE_NAME "explore_node"

using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{

  public:
    MapGenerator(const std::string& mapname, const std::string& path, int threshold_occupied, int threshold_free)
      : mapname_(mapname), path_(path), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);


      std::string mapdatafile = path_ + mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: explore_node.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile = path_ + mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
       */

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              std::string(mapname_ + ".pgm").c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    std::string mapname_;
    std::string path_;
    ros::Subscriber map_sub_;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;

};


using namespace rt_gui;

void saveExploredMap(std::string mapname)
{

  std::string path = "/tmp/";
  int threshold_occupied = 65;
  int threshold_free = 25;
  MapGenerator mg(mapname, path, threshold_occupied, threshold_free);

  while(!mg.saved_map_ && ros::ok())
    ros::spinOnce();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  //if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                   ros::console::levels::Debug)) {
  //  ros::console::notifyLoggerLevelsChanged();
  //}

  explore::Explore explore_map;
  explore::Costmap2DServer costmap_server;

  // create interface
  ros::param::set("/explore/save_map_to","map");
  if(RtGuiClient::getIstance().init("wolf_panel","explore"))
  {
    RtGuiClient::getIstance().addTrigger(std::string("explore"),std::string("Start"),std::bind(&explore::Explore::start,&explore_map));
    RtGuiClient::getIstance().addTrigger(std::string("explore"),std::string("Stop"),std::bind(&explore::Explore::stop,&explore_map));
    RtGuiClient::getIstance().addText(std::string("explore"),std::string("save_map_to"),&saveExploredMap,false);
  }

  std::thread exploration_loop(std::bind(&explore::Explore::makePlan,&explore_map));

  costmap_server.start();

  ros::Rate rate(50.0);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();
    rate.sleep();
  }

  exploration_loop.join();
  costmap_server.stop();

  return 0;
}
