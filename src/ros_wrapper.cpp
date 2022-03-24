#include "wolf_navigation/ros_wrapper.h"

using namespace wolf_navigation;

Eigen::Vector3d rotToRpy(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d rpy;
  rpy(0) = std::atan2(R(2,1),R(2,2));
  rpy(1) = std::atan2(-R(2,0),std::sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
  rpy(2) = std::atan2(R(1,0),R(0,0));
  return rpy;
}

Eigen::Matrix3d rpyToRot(const Eigen::Vector3d& rpy)
{
  Eigen::Matrix3d R;

  R.setZero();

  double c_y = std::cos(rpy(2));
  double s_y = std::sin(rpy(2));

  double c_r = std::cos(rpy(0));
  double s_r = std::sin(rpy(0));

  double c_p = std::cos(rpy(1));
  double s_p = std::sin(rpy(1));

  R << c_p*c_y ,  s_r*s_p*c_y - c_r*s_y                 ,  c_r*s_p*c_y + s_r*s_y  ,
       c_p*s_y ,  s_r*s_p*s_y + c_r*c_y                 ,  s_y*s_p*c_r - c_y*s_r,
       -s_p    ,  c_p*s_r                               ,  c_r*c_p;

  return R;
}

RosWrapper::RosWrapper(ros::NodeHandle &nh)
  : tf_listener_(tf_buffer_)
{
  t_ = t_prev_ = ros::Time::now();
  nh_ = nh;
}

void RosWrapper::init(const std::vector<std::string>& trackingcamera_topics, const std::string& child_frame_id, bool twist_in_local_frame)
{
  //trackingcamera_topic_ = trackingcamera_topic;

  child_frame_id_ = child_frame_id;
  odom_frame_id_ = "odom";  // FIXME
  odom_topic_    = "/odom"; // FIXME

  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_,20);

  size_t n_trackingcameras = trackingcamera_topics.size();

  if(n_trackingcameras == 1)
  {
    single_camera_sub_ = nh_.subscribe(trackingcamera_topics[0],20,&RosWrapper::singleCameraCallback,this);
    camera_estimators_.push_back(std::make_shared<TrackingCameraEstimator>(twist_in_local_frame));
  }
  else if (n_trackingcameras == 2)
  {
      double_camera_0_sub_.subscribe(nh_,trackingcamera_topics[0],20);
      double_camera_1_sub_.subscribe(nh_,trackingcamera_topics[1],20);
      double_camera_sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::Odometry,nav_msgs::Odometry> >(double_camera_0_sub_,double_camera_1_sub_,20);
      double_camera_sync_->registerCallback(boost::bind(&RosWrapper::doubleCameraCallback,this,_1,_2));
      camera_estimators_.push_back(std::make_shared<TrackingCameraEstimator>(twist_in_local_frame)); // 0
      camera_estimators_.push_back(std::make_shared<TrackingCameraEstimator>(twist_in_local_frame)); // 1
  }
  else
      throw std::runtime_error("RosWrapper supports maximum 2 tracking cameras!");

}

void RosWrapper::updateCamera(const unsigned int &camera_id, const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  tmp_isometry3d_.translation() = Eigen::Vector3d(odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z);
  tmp_isometry3d_.linear() = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z).toRotationMatrix();
  tf2::fromMsg(odom_msg->twist.twist.linear, tmp_vector3d_);
  tf2::fromMsg(odom_msg->twist.twist.angular,tmp_vector3d_1_);
  camera_estimators_[camera_id]->setCameraLinearTwist(tmp_vector3d_);
  camera_estimators_[camera_id]->setCameraAngularTwist(tmp_vector3d_1_);
  camera_estimators_[camera_id]->setCameraPose(tmp_isometry3d_);
  camera_estimators_[camera_id]->update();
}

void RosWrapper::publish(const Eigen::Isometry3d &pose, const Eigen::Matrix6d &pose_cov, const Eigen::Vector6d twist, const Eigen::Matrix6d &twist_cov)
{
  // Get current ROS time
  t_ = ros::Time::now();

  if(t_ != t_prev_)
  {
    // Publish TF msg
    // The navigation stack uses tf to determine the robot's location in the world and relate sensor data to a static map.
    transform_msg_out_ = tf2::eigenToTransform(pose);
    transform_msg_out_.header.seq++;
    transform_msg_out_.header.stamp = t_;
    transform_msg_out_.header.frame_id = odom_frame_id_;
    transform_msg_out_.child_frame_id = child_frame_id_;
    tf_broadcaster_.sendTransform(transform_msg_out_);

    // Publish odom msg
    // The nav_msgs/Odometry message stores an estimate of the position and velocity of a robot in free space:
    // This represents an estimate of a position and velocity in free space.
    // The pose in this message should be specified in the coordinate frame given by header.frame_id (in our case odom)
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id (in our case the basefootprint)
    odom_msg_out_.header                = transform_msg_out_.header;
    odom_msg_out_.child_frame_id        = transform_msg_out_.child_frame_id;
    odom_msg_out_.pose.pose.position.x  = transform_msg_out_.transform.translation.x;
    odom_msg_out_.pose.pose.position.y  = transform_msg_out_.transform.translation.y;
    odom_msg_out_.pose.pose.position.z  = transform_msg_out_.transform.translation.z;
    odom_msg_out_.pose.pose.orientation = transform_msg_out_.transform.rotation;

    odom_msg_out_.twist.twist = tf2::toMsg(twist);

    //odom_msg_.twist                  = odom_T_trackingcamera->twist;
    //odom_msg_.twist.twist.linear.z   = 0.0; // basefootprint is on the ground
    //odom_msg_.twist.twist.angular.x  = 0.0; // no roll
    //odom_msg_.twist.twist.angular.y  = 0.0; // no pitch
    //odom_msg_out_.twist.covariance      =
    //odom_msg_out_.pose.covariance       =
    // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
    // FIXME Now we are using the same covariance of the trackingcamera, we need to adapt it!
    odom_publisher_.publish(odom_msg_out_);
  }
  t_prev_ = t_;
}

void RosWrapper::doubleCameraCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_0, const nav_msgs::Odometry::ConstPtr& odom_msg_1)
{
  try
  {
    camera_estimators_[0]->setBaseCameraTransform(tf2::transformToEigen(tf_buffer_.lookupTransform(odom_msg_0->child_frame_id,child_frame_id_,ros::Time(0))));
    camera_estimators_[1]->setBaseCameraTransform(tf2::transformToEigen(tf_buffer_.lookupTransform(odom_msg_1->child_frame_id,child_frame_id_,ros::Time(0))));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  // Camera 0
  updateCamera(0,odom_msg_0);

  // Camera 1
  updateCamera(1,odom_msg_1);

  // Smoothing filter pose
  tmp_vector3d_   = (camera_estimators_[0]->getBasePose().translation() + camera_estimators_[1]->getBasePose().translation())/2.0; // xyz
  tmp_vector3d_1_ = (rotToRpy(camera_estimators_[0]->getBasePose().linear()) + rotToRpy(camera_estimators_[1]->getBasePose().linear()))/2.0; // rpy
  tmp_isometry3d_.translation() = tmp_vector3d_;
  tmp_isometry3d_.linear() = rpyToRot(tmp_vector3d_1_);
  // Smoothing filter twist
  tmp_vector6d_   = (camera_estimators_[0]->getBaseTwist() + camera_estimators_[1]->getBaseTwist())/2.0;

  publish(tmp_isometry3d_,Eigen::Matrix6d::Zero(),tmp_vector6d_,Eigen::Matrix6d::Zero());
}

void RosWrapper::singleCameraCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  try
  {
    camera_estimators_[0]->setBaseCameraTransform(tf2::transformToEigen(tf_buffer_.lookupTransform(odom_msg->child_frame_id,child_frame_id_,ros::Time(0))));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  updateCamera(0,odom_msg);

  tmp_isometry3d_ = camera_estimators_[0]->getBasePose();
  tmp_vector6d_   = camera_estimators_[0]->getBaseTwist();

  publish(tmp_isometry3d_,Eigen::Matrix6d::Zero(),tmp_vector6d_,Eigen::Matrix6d::Zero());
}
