/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
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
#ifdef RT_GUI
#include <rt_gui/rt_gui_client.h>
#endif
#include <ros/ros.h>
#include <artifacts_mapping/addClass.h>
#include <std_srvs/Empty.h>

#define NODE_NAME "artifacts_search_node"

using namespace wolf_exploration;

static ArtifactsSearch::Ptr _artifacts_search;

static std::string _add_class_srv = "/artifacts_manager/add_class";
static std::string _clear_classes_srv = "/artifacts_manager/clear_classes";
static int _timeout = 10;

void searchArtifact(std::string artifact_name)
{
  artifacts_mapping::addClass srv;
  srv.request.value = artifact_name;
  if(ros::service::waitForService(_add_class_srv,_timeout))
  {
    ros::service::call(_add_class_srv,srv);
    _artifacts_search->start();
  }
  else
    ROS_WARN_NAMED(NODE_NAME,"ROS service %s has not been advertised yet!",_add_class_srv.c_str());
}

void stop()
{
  _artifacts_search->pause();
}

void reset()
{
  std_srvs::Empty srv;
  if(ros::service::waitForService(_clear_classes_srv,_timeout))
  {
    ros::service::call(_clear_classes_srv,srv);
    _artifacts_search->stop();
  }
  else
    ROS_WARN_NAMED(NODE_NAME,"ROS service %s has not been advertised yet!",_clear_classes_srv.c_str());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  //if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                   ros::console::levels::Debug)) {
  //  ros::console::notifyLoggerLevelsChanged();
  //}

  Costmap2DServer costmap_server("artifacts_costmap");
  costmap_server.start();

  _artifacts_search.reset(new ArtifactsSearch());

  // create interface
  #ifdef RT_GUI
  ros::param::set("/artifacts_search/artifact_name","artifact");
  if(rt_gui::RtGuiClient::getIstance().init("wolf_panel","artifacts_search"))
  {
    rt_gui::RtGuiClient::getIstance().addText(std::string("artifacts_search"),std::string("artifact_name"),&searchArtifact,false);
    //RtGuiClient::getIstance().addTrigger(std::string("artifacts_search"),std::string("Start"),std::bind(&ArtifactsSearch::start,&artifacts_search));
    rt_gui::RtGuiClient::getIstance().addTrigger(std::string("artifacts_search"),std::string("Stop"),stop);
    rt_gui::RtGuiClient::getIstance().addTrigger(std::string("artifacts_search"),std::string("Reset"),reset);
  }
  #endif

  std::thread artifacts_search_loop(std::bind(&ArtifactsSearch::makePlan,_artifacts_search));

  ros::Rate rate(50.0);
  while(ros::ok())
  {
    #ifdef RT_GUI
    rt_gui::RtGuiClient::getIstance().sync();
    #endif
    rate.sleep();
  }

  costmap_server.stop();

  artifacts_search_loop.join();

  return 0;
}
