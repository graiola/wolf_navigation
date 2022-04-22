#include "wolf_navigation/waypoints.h"

Waypoints::Waypoints(ros::NodeHandle &nh)
  :nh_(nh)
{
  interactive_marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("waypoints");
}

void Waypoints::addWaypoint(const move_base_msgs::MoveBaseGoal& goal)
{
  std::lock_guard<std::mutex> lk(mtx_);

  unsigned int id = list_.size()+1;

  tf2::Quaternion q;
  tf2::fromMsg(goal.target_pose.pose.orientation,q);
  tf2::Matrix3x3 mat(q);
  tf2Scalar roll, pitch, yaw;
  mat.getEulerYPR(yaw,pitch,roll);

  ROS_INFO_NAMED(CLASS_NAME,"Add waypoint %i -> [x: %f, y: %f, z: %f, r: %f, p: %f, y: %f]",
                 id,goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z,roll,pitch,yaw);


  list_.push_back(std::make_shared<Waypoint>(id,goal));

  // Visualize waypoint
  interactive_marker_server_->insert(list_.back()->getInteractiveMarker());
  list_.back()->createMenuEntry(boost::bind(&Waypoints::removeWaypoint,this,list_.back()->getId()),*interactive_marker_server_);
  interactive_marker_server_->applyChanges();
}

void Waypoints::addWaypoint(const geometry_msgs::PoseStamped& pose)
{
  move_base_msgs::MoveBaseGoal goal;
  // fill in the navigation goal message
  goal.target_pose.header.frame_id    = pose.header.frame_id;
  goal.target_pose.header.stamp       = pose.header.stamp;
  goal.target_pose.pose.position.x    = pose.pose.position.x;
  goal.target_pose.pose.position.y    = pose.pose.position.y;
  goal.target_pose.pose.position.z    = 0.0;
  goal.target_pose.pose.orientation.x = pose.pose.orientation.x;
  goal.target_pose.pose.orientation.y = pose.pose.orientation.y;
  goal.target_pose.pose.orientation.z = pose.pose.orientation.z;
  goal.target_pose.pose.orientation.w = pose.pose.orientation.w;
  addWaypoint(goal);
}

void Waypoints::addWaypoint(const geometry_msgs::PointStamped& point)
{
  mtx_.lock();

  unsigned int id_waypoint = list_.size()+1;

  move_base_msgs::MoveBaseGoal goal;
  // fill in the navigation goal message
  goal.target_pose.header.frame_id = point.header.frame_id;
  goal.target_pose.header.stamp    = point.header.stamp;
  goal.target_pose.pose.position.x = point.point.x;
  goal.target_pose.pose.position.y = point.point.y;
  goal.target_pose.pose.position.z = 0.0;

  // align the robot with the heading of the current waypoint wrt the previous one
  double yaw = 0.0;
  if(list_.size()!=0)
  {
    for(unsigned int i=0;i<list_.size();i++)
      if(list_[i]->getId() == id_waypoint - 1)
        yaw = std::atan2(point.point.y-list_[i]->getGoal().target_pose.pose.position.y,point.point.x-list_[i]->getGoal().target_pose.pose.position.x);
  }
  else
    yaw = std::atan2(point.point.y,point.point.x);

  tf2::Quaternion q;
  q.setRPY( 0., 0., yaw );
  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  mtx_.unlock();

  addWaypoint(goal);
}

move_base_msgs::MoveBaseGoal Waypoints::getCurrentWaypointGoal()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return list_[0]->getGoal(); // FIXME
}

unsigned int Waypoints::getCurrentWaypointId()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return list_[0]->getId(); // FIXME
}

void Waypoints::removeCurrentWaypoint()
{
  std::lock_guard<std::mutex> lk(mtx_);
  ROS_INFO_NAMED(CLASS_NAME,"Remove waypoint %i ", list_.front()->getId());
  interactive_marker_server_->erase(list_.front()->getName());
  interactive_marker_server_->applyChanges();
  list_.pop_front();
}

void Waypoints::removeWaypoint(const unsigned int &id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ROS_INFO_NAMED(CLASS_NAME,"Remove waypoint %i ", id);
  interactive_marker_server_->erase(list_[id-1]->getName());
  interactive_marker_server_->applyChanges();
  list_.erase(list_.begin()+id-1);
}

void Waypoints::moveToNextWaypoint()
{
  std::lock_guard<std::mutex> lk(mtx_);
  std::rotate(list_.begin(), list_.begin() + 1, list_.end());
}

unsigned int Waypoints::getNumberOfWaypoints()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return list_.size();
}
