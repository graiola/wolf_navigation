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

#include "wolf_exploration/artifacts_search.h"
#include "wolf_exploration/costmap_server.h"

#include <thread>
#include <rt_gui/rt_gui_client.h>
#include <ros/ros.h>

#define NODE_NAME "artifacts_search_node"

using namespace rt_gui;
using namespace wolf_exploration;

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  //if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                   ros::console::levels::Debug)) {
  //  ros::console::notifyLoggerLevelsChanged();
  //}

  Costmap2DServer costmap_server("artifacts_costmap");
  costmap_server.start();

  ArtifactsSearch artifacts_search;

  // create interface
  if(RtGuiClient::getIstance().init("wolf_panel","artifacts_search"))
  {
    RtGuiClient::getIstance().addTrigger(std::string("artifacts_search"),std::string("Start"),std::bind(&ArtifactsSearch::start,&artifacts_search));
    RtGuiClient::getIstance().addTrigger(std::string("artifacts_search"),std::string("Stop"),std::bind(&ArtifactsSearch::stop,&artifacts_search));
  }

  std::thread artifacts_search_loop(std::bind(&ArtifactsSearch::makePlan,&artifacts_search));

  ros::Rate rate(50.0);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();
    rate.sleep();
  }

  costmap_server.stop();

  artifacts_search_loop.join();

  return 0;
}
