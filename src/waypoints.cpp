#include "wolf_navigation/waypoints.h"

Waypoints::Waypoints(ros::NodeHandle &nh)
  :nh_(nh)
{
  marker_.reset(new WaypointMarker(nh));
}

void Waypoints::addWaypoint(const move_base_msgs::MoveBaseGoal& waypoint)
{
  std::lock_guard<std::mutex> lk(mtx_);

  unsigned int id_waypoint = list_.size()+1;

  tf2::Quaternion q;
  tf2::fromMsg(waypoint.target_pose.pose.orientation,q);
  tf2::Matrix3x3 mat(q);
  tf2Scalar roll, pitch, yaw;
  mat.getEulerYPR(yaw,pitch,roll);

  ROS_INFO_NAMED(CLASS_NAME,"Add waypoint %i -> [x: %f, y: %f, z: %f, r: %f, p: %f, y: %f]",
                 id_waypoint,waypoint.target_pose.pose.position.x,waypoint.target_pose.pose.position.y,waypoint.target_pose.pose.position.z,roll,pitch,yaw);

  list_.push_back(std::pair<unsigned int,move_base_msgs::MoveBaseGoal>(id_waypoint,waypoint));

  // Visualize waypoint
  marker_->addMarker(id_waypoint,waypoint);
}

void Waypoints::addWaypoint(const geometry_msgs::PoseStamped& waypoint)
{
  move_base_msgs::MoveBaseGoal goal;
  // fill in the navigation goal message
  goal.target_pose.header.frame_id    = waypoint.header.frame_id;
  goal.target_pose.header.stamp       = waypoint.header.stamp;
  goal.target_pose.pose.position.x    = waypoint.pose.position.x;
  goal.target_pose.pose.position.y    = waypoint.pose.position.y;
  goal.target_pose.pose.position.z    = 0.0;
  goal.target_pose.pose.orientation.x = waypoint.pose.orientation.x;
  goal.target_pose.pose.orientation.y = waypoint.pose.orientation.y;
  goal.target_pose.pose.orientation.z = waypoint.pose.orientation.z;
  goal.target_pose.pose.orientation.w = waypoint.pose.orientation.w;
  addWaypoint(goal);
}

void Waypoints::addWaypoint(const geometry_msgs::PointStamped& waypoint)
{
  mtx_.lock();

  unsigned int id_waypoint = list_.size()+1;

  move_base_msgs::MoveBaseGoal goal;
  // fill in the navigation goal message
  goal.target_pose.header.frame_id = waypoint.header.frame_id;
  goal.target_pose.header.stamp    = waypoint.header.stamp;
  goal.target_pose.pose.position.x = waypoint.point.x;
  goal.target_pose.pose.position.y = waypoint.point.y;
  goal.target_pose.pose.position.z = 0.0;

  // align the robot with the heading of the current waypoint wrt the previous one
  double yaw = 0.0;
  if(list_.size()!=0)
  {
    for(unsigned int i=0;i<list_.size();i++)
      if(list_[i].first == id_waypoint - 1)
        yaw = std::atan2(waypoint.point.y-list_[i].second.target_pose.pose.position.y,waypoint.point.x-list_[i].second.target_pose.pose.position.x);
  }
  else
    yaw = std::atan2(waypoint.point.y,waypoint.point.x);

  tf2::Quaternion q;
  q.setRPY( 0., 0., yaw );
  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  mtx_.unlock();

  addWaypoint(goal);
}

move_base_msgs::MoveBaseGoal Waypoints::getCurrentWaypoint()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return list_[0].second; // FIXME
}

unsigned int Waypoints::getCurrentWaypointId()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return list_[0].first; // FIXME
}

void Waypoints::removeCurrentWaypoint()
{
  std::lock_guard<std::mutex> lk(mtx_);
  marker_->removeMarker(list_.front().first);
  list_.pop_front();
}

void Waypoints::removeWaypoint(const unsigned int &id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  // TODO
}

void Waypoints::moveToNextWaypoint()
{
  std::lock_guard<std::mutex> lk(mtx_);


  //std::cout << "BEFORE" << std::endl;
  //for(unsigned int i=0;i<list_.size();i++)
  //  std::cout << "w " << list_[i].first << std::endl;

  std::rotate(list_.begin(), list_.begin() + 1, list_.end());

  //std::cout << "AFTER" << std::endl;
  //for(unsigned int i=0;i<list_.size();i++)
  //  std::cout << "w " << list_[i].first << std::endl;

}

unsigned int Waypoints::getNumberOfWaypoints()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return list_.size();
}
