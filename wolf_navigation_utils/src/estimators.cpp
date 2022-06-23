#include "wolf_navigation_utils/estimators.h"

using namespace wolf_navigation;

void TrackingCameraEstimator::update()
{

  odom_T_base_ = odom_T_trackingcamera_ * trackingcamera_T_base_;

  base_R_odom_ = odom_T_base_.linear().transpose();
  base_R_trackingcamera_ = trackingcamera_T_base_.linear().transpose();

  if(!twist_in_local_frame_) // In this case the twist from the trackingcamera is defined wrt the odom frame
  {
    // base_v = base_R_odom * odom_v_trackingcamera + base_S_odom * base_R_odom * odom_p_trackingcamera
    // base_omega = base_R_odom * odom_omega_trackingcamera
    base_twist_.head(3) = base_R_odom_ * trackingcamera_twist_.head(3); // TODO + base_S_odom * base_R_odom * odom_p_trackingcamera
    base_twist_.tail(3) = base_R_odom_ * trackingcamera_twist_.tail(3);

    base_twist_cov_.topLeftCorner(3,3) = base_R_odom_ * trackingcamera_twist_cov_.topLeftCorner(3,3) * base_R_odom_.transpose(); // xyz
    //base_twist_cov_.bottomRightCorner(3,3) = base_R_odom_ * trackingcamera_twist_cov_.bottomRightCorner(3,3) * base_R_odom_.transpose(); // rpy
  }
  else  // In this case the twist from the trackingcamera is defined wrt to itself
  {
    // base_v = base_R_trackingcamera * trackingcamera_v + base_S_trackingcamera * base_R_trackingcamera * trackingcamera_p
    // base_omega = base_R_trackingcamera * trackingcamera_omega
    base_twist_.head(3) = base_R_trackingcamera_ * trackingcamera_twist_.head(3); // TODO + base_S_trackingcamera * base_R_trackingcamera * trackingcamera_p
    base_twist_.tail(3) = base_R_trackingcamera_ * trackingcamera_twist_.tail(3);

    base_twist_cov_.topLeftCorner(3,3) = base_R_trackingcamera_ * trackingcamera_twist_cov_.topLeftCorner(3,3) * base_R_trackingcamera_.transpose(); // xyz
    //base_twist_cov_.bottomRightCorner(3,3) = base_R_trackingcamera_ * trackingcamera_twist_cov_.bottomRightCorner(3,3) * base_R_trackingcamera_.transpose(); // rpy
  }

  base_pose_cov_.topLeftCorner(3,3) = base_R_trackingcamera_ * trackingcamera_pose_cov_.topLeftCorner(3,3) * base_R_trackingcamera_.transpose(); // xyz
  //base_pose_cov_.bottomRightCorner(3,3) = base_R_trackingcamera_ * trackingcamera_pose_cov_.bottomRightCorner(3,3) * base_R_trackingcamera_.transpose(); // rpy // TODO is that correct?

  odom_X_base_.head(3) = odom_T_base_.translation();
  odom_X_base_.tail(3) = rotToRpy(odom_T_base_.linear());
}

TrackingCameraEstimator::TrackingCameraEstimator(bool twist_in_local_frame)
{
  twist_in_local_frame_                 = twist_in_local_frame;
  base_twist_ = trackingcamera_twist_   = Eigen::Vector6d::Zero();
  base_R_odom_ = base_R_trackingcamera_ = Eigen::Matrix3d::Identity();
  odom_T_base_ = odom_T_trackingcamera_ = trackingcamera_T_base_ = Eigen::Isometry3d::Identity();
  base_pose_cov_ = base_twist_cov_ = trackingcamera_pose_cov_ = trackingcamera_twist_cov_ = Eigen::Matrix6d::Identity();
}

void TrackingCameraEstimator::setBaseCameraTransform(const Eigen::Isometry3d &trackingcamera_T_base)
{
  trackingcamera_T_base_ = trackingcamera_T_base;
}

void TrackingCameraEstimator::setCameraPose(const Eigen::Isometry3d &odom_T_trackingcamera)
{
  odom_T_trackingcamera_ = odom_T_trackingcamera;
}

void TrackingCameraEstimator::setCameraAngularTwist(const Eigen::Vector3d &trackingcamera_omega)
{
  trackingcamera_twist_.tail(3) = trackingcamera_omega;
}

void TrackingCameraEstimator::setCameraPoseCovariance(const Eigen::Matrix6d &cov)
{
  trackingcamera_pose_cov_ = cov;
}

void TrackingCameraEstimator::setCameraTwistCovariance(const Eigen::Matrix6d &cov)
{
  trackingcamera_twist_cov_ = cov;
}

void TrackingCameraEstimator::setCameraLinearTwist(const Eigen::Vector3d &trackingcamera_v)
{
  trackingcamera_twist_.head(3) = trackingcamera_v;
}

void TrackingCameraEstimator::setCameraTwist(const Eigen::Vector6d &trackingcamera_twist)
{
  trackingcamera_twist_ = trackingcamera_twist;
}

const Eigen::Isometry3d &TrackingCameraEstimator::getBasePose()
{
  return odom_T_base_;
}

const Eigen::Vector6d &TrackingCameraEstimator::getBasePose6d()
{
  return odom_X_base_;
}

Eigen::Vector3d TrackingCameraEstimator::getBaseLinearTwist()
{
  return base_twist_.head(3);
}

Eigen::Vector3d TrackingCameraEstimator::getBaseAngularTwist()
{
  return base_twist_.tail(3);
}

Eigen::Matrix6d TrackingCameraEstimator::getBasePoseCovariance()
{
  return base_pose_cov_;
}

Eigen::Matrix6d TrackingCameraEstimator::getBaseTwistCovariance()
{
  return base_twist_cov_;
}

const Eigen::Vector6d &TrackingCameraEstimator::getBaseTwist()
{
  return base_twist_;
}


BasefootEstimator::BasefootEstimator()
{
  base_T_basefoot_.setIdentity();
  odom_T_basefoot_.setIdentity();
  odom_T_base_.setIdentity();
  basefoot_T_odom_.setIdentity();
}

void BasefootEstimator::setContacts(const std::vector<bool> &foot_contacts, std::vector<double> &foot_height)
{
  if(foot_height_.size() == foot_contacts_.size())
  {
    foot_contacts_ = foot_contacts;
    foot_height_ = foot_height;
  }
}

void BasefootEstimator::setBaseInOdom(const Eigen::Isometry3d& pose)
{
  odom_T_base_ = pose;
}

const Eigen::Isometry3d &BasefootEstimator::getBasefootInBase()
{
  return base_T_basefoot_;
}

const Eigen::Isometry3d &BasefootEstimator::getBasefootInOdom()
{
  return odom_T_basefoot_;
}

void BasefootEstimator::update()
{
  // Reset
  base_T_basefoot_.setIdentity();
  odom_T_basefoot_.setIdentity();
  odom_T_base_.setIdentity();
  basefoot_T_odom_.setIdentity();
  tmp_v_.setZero();
  tmp_R_.setIdentity();

  // Create base_T_basefoot
  base_T_basefoot_.translation().x() = 0.0;
  base_T_basefoot_.translation().y() = 0.0;
  base_T_basefoot_.translation().z() = -estimateHeight();

  // Create odom_T_basefoot
  tmp_v_ =  odom_T_base_.translation();
  tmp_v_(2) = tmp_v_(2) + base_T_basefoot_.translation().z();
  double base_yaw   = std::atan2(odom_T_base_.linear()(1,0),odom_T_base_.linear()(0,0));
  rpyToRotTranspose(0.0,0.0,base_yaw,tmp_R_);
  tmp_v_ = - tmp_R_ * tmp_v_;
  basefoot_T_odom_.translation().x() = tmp_v_(0);
  basefoot_T_odom_.translation().y() = tmp_v_(1);
  basefoot_T_odom_.translation().z() = tmp_v_(2);
  basefoot_T_odom_.linear()          = tmp_R_;
  odom_T_basefoot_ = basefoot_T_odom_.inverse();
}

double BasefootEstimator::estimateHeight()
{
  // Estimate z using the foot heights
  double estimated_z = 0.0;
  int feet_in_stance = 0;
  for(unsigned int i = 0; i<foot_contacts_.size(); i++)
  {
    if(foot_contacts_[i])
    {
      feet_in_stance++;
      estimated_z += foot_height_[i];
    }
  }
  estimated_z /= feet_in_stance;
  return estimated_z;
}
