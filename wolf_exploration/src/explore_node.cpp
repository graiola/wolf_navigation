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

#include <thread>
#include <rt_gui/rt_gui_client.h>
#include <ros/ros.h>
#include <wolf_navigation_utils/map_saver.h>

#define NODE_NAME "explore_node"

using namespace rt_gui;
using namespace wolf_navigation_utils;

void saveExploredMap(std::string mapname)
{
  std::string path = "/tmp/";
  int threshold_occupied = 65;
  int threshold_free = 25;
  MapSaver map_saver(mapname, path, threshold_occupied, threshold_free);

  while(!map_saver.mapSaved() && ros::ok())
    ros::spinOnce();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  //if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                   ros::console::levels::Debug)) {
  //  ros::console::notifyLoggerLevelsChanged();
  //}

  explore::Explore explore;

  // create interface
  ros::param::set("/explore/save_map_to","map");
  if(RtGuiClient::getIstance().init("wolf_panel","explore"))
  {
    RtGuiClient::getIstance().addTrigger(std::string("explore"),std::string("Start"),std::bind(&explore::Explore::start,&explore));
    RtGuiClient::getIstance().addTrigger(std::string("explore"),std::string("Stop"),std::bind(&explore::Explore::stop,&explore));
    RtGuiClient::getIstance().addText(std::string("explore"),std::string("save_map_to"),&saveExploredMap,false);
  }

  std::thread exploration_loop(std::bind(&explore::Explore::makePlan,&explore));

  ros::Rate rate(50.0);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();
    rate.sleep();
  }

  exploration_loop.join();

  return 0;
}
