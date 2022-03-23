#include "wolf_navigation/estimators.h"

using namespace wolf_navigation;

void TrackingCameraEstimator::update()
{

  odom_T_base_ = odom_T_trackingcamera_ * trackingcamera_T_base_;

  if(!twist_in_local_frame_) // In this case the twist from the trackingcamera is defined wrt the odom frame
  {
    // base_v = base_R_odom * odom_v_trackingcamera + base_S_odom * base_R_odom * odom_p_trackingcamera
    // base_omega = base_R_odom * odom_omega_trackingcamera
    base_R_odom_ = odom_T_base_.linear().transpose();
    base_v_ = base_R_odom_ * trackingcamera_v_; // TODO + base_S_odom * base_R_odom * odom_p_trackingcamera
    base_omega_ = base_R_odom_ * trackingcamera_omega_;
  }
  else  // In this case the twist from the trackingcamera is defined wrt to itself
  {
    // base_v = base_R_trackingcamera * trackingcamera_v + base_S_trackingcamera * base_R_trackingcamera * trackingcamera_p
    // base_omega = base_R_trackingcamera * trackingcamera_omega
    base_R_trackingcamera_ = trackingcamera_T_base_.linear().transpose();
    base_v_ = base_R_trackingcamera_ * trackingcamera_v_; // TODO + base_S_trackingcamera * base_R_trackingcamera * trackingcamera_p
    base_omega_ = base_R_trackingcamera_ * trackingcamera_omega_;
  }

}

TrackingCameraEstimator::TrackingCameraEstimator(bool twist_in_local_frame)
{
  twist_in_local_frame_ = twist_in_local_frame;
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
  trackingcamera_omega_ = trackingcamera_omega;
}

void TrackingCameraEstimator::setCameraLinearTwist(const Eigen::Vector3d &trackingcamera_v)
{
  trackingcamera_v_ = trackingcamera_v;
}

const Eigen::Isometry3d &TrackingCameraEstimator::getBasePose()
{
  return odom_T_base_;
}

const Eigen::Vector3d &TrackingCameraEstimator::getBaseLinearTwist()
{
  return base_v_;
}

const Eigen::Vector3d &TrackingCameraEstimator::getBaseAngularTwist()
{
  return base_omega_;
}


