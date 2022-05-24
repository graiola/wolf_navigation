#include "wolf_exploration/costmap_server.h"


using namespace costmap_2d;
using namespace tf2_ros;

namespace wolf_exploration
{

Costmap2DServer::Costmap2DServer(const std::string& costmap_name)
{
  buffer_ = std::make_shared<Buffer>(ros::Duration(10));
  tf_ = std::make_shared<TransformListener>(*buffer_.get());

  costmap_ = std::make_shared<Costmap2DROS>(costmap_name,*buffer_.get());
  costmap_->pause();
}

void Costmap2DServer::start()
{
  // Start actively updating costmaps based on sensor data
  costmap_->start();
}

void Costmap2DServer::stop()
{
  costmap_->stop();
}

void Costmap2DServer::pause()
{
  costmap_->pause();
}

}  // namespace explore
