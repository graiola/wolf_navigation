#include "wolf_navigation/estimators.h"

using namespace wolf_navigation;

void TrackingCameraEstimator::update(bool twist_in_local_frame)
{

    // The tracking camera provides us the following transform: odom_T_trackingcamera
    // We need to report everything wrt the robot's base footprint i.e.: odom_T_basefootprint = odom_T_trackingcamera * trackingcamera_T_basefootprint
    odom_T_basefootprint_ = odom_T_trackingcamera_ * trackingcamera_T_basefootprint_;


    if(!twist_in_local_frame) // In this case the twist from the trackingcamera is defined wrt the odom frame
    {
        // basefootprint_v = basefootprint_R_odom * odom_v_trackingcamera + basefootprint_S_odom * basefootprint_R_odom * odom_p_trackingcamera
        // basefootprint_omega = basefootprint_R_odom * odom_omega_trackingcamera
        basefootprint_R_odom_ = odom_T_basefootprint_.linear().transpose();
        basefootprint_v_ = basefootprint_R_odom_ * trackingcamera_v_; // TODO + basefootprint_S_odom * basefootprint_R_odom * odom_p_trackingcamera
        basefootprint_omega_ = basefootprint_R_odom_ * trackingcamera_omega_;
    }
    else  // In this case the twist from the trackingcamera is defined wrt to itself
    {
        // basefootprint_v = basefootprint_R_trackingcamera * trackingcamera_v + basefootprint_S_trackingcamera * basefootprint_R_trackingcamera * trackingcamera_p
        // basefootprint_omega = basefootprint_R_trackingcamera * trackingcamera_omega
        basefootprint_R_trackingcamera_ = trackingcamera_T_basefootprint_.linear().transpose();
        basefootprint_v_ = basefootprint_R_trackingcamera_ * trackingcamera_v_; // TODO + basefootprint_S_trackingcamera * basefootprint_R_trackingcamera * trackingcamera_p
        basefootprint_omega_ = basefootprint_R_trackingcamera_ * trackingcamera_omega_;
    }

}

void TrackingCameraEstimator::setTransform(const Eigen::Isometry3d &transform)
{
    trackingcamera_T_basefootprint_ = transform;
}

void TrackingCameraEstimator::setPose(const Eigen::Isometry3d &pose)
{
    odom_T_trackingcamera_ = pose;
}

void TrackingCameraEstimator::setAngularTwist(const Eigen::Vector3d &angular)
{
    trackingcamera_omega_ = angular;
}

const Eigen::Isometry3d &TrackingCameraEstimator::getPose()
{
    return odom_T_basefootprint_;
}

const Eigen::Vector3d &TrackingCameraEstimator::getLinearTwist()
{
    return basefootprint_v_;
}

const Eigen::Vector3d &TrackingCameraEstimator::getAngularTwist()
{
    return basefootprint_omega_;
}

void TrackingCameraEstimator::setLinearTwist(const Eigen::Vector3d &linear)
{
    trackingcamera_v_ = linear;
}
