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

#ifndef WOLF_NAVIGATION_UTILS_ESTIMATOR_H
#define WOLF_NAVIGATION_UTILS_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <iostream>

namespace Eigen
{
    typedef Matrix<double,6,1> Vector6d;
    typedef Matrix<double,7,1> Vector7d;
    typedef Matrix<double,6,6> Matrix6d;
    typedef Matrix<double,4,3> Matrix4x3d;
    typedef Matrix<double,3,6> Matrix3x6d;
}

inline Eigen::Vector3d rotToRpy(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d rpy;
  rpy(0) = std::atan2(R(2,1),R(2,2));
  rpy(1) = std::atan2(-R(2,0),std::sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
  rpy(2) = std::atan2(R(1,0),R(0,0));
  return rpy;
}

inline Eigen::Matrix3d rpyToRot(const Eigen::Vector3d& rpy)
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

inline Eigen::Matrix3d rpyToRot(const double& roll, const double& pitch, const double& yaw, Eigen::Matrix3d& R)
{
  R.setZero();

  double c_y = std::cos(yaw);
  double s_y = std::sin(yaw);

  double c_r = std::cos(roll);
  double s_r = std::sin(roll);

  double c_p = std::cos(pitch);
  double s_p = std::sin(pitch);

  R << c_p*c_y ,  s_r*s_p*c_y - c_r*s_y                 ,  c_r*s_p*c_y + s_r*s_y  ,
       c_p*s_y ,  s_r*s_p*s_y + c_r*c_y                 ,  s_y*s_p*c_r - c_y*s_r,
       -s_p    ,  c_p*s_r                               ,  c_r*c_p;

  return R;
}

inline void rpyToRotTranspose(const double& roll, const double& pitch, const double& yaw, Eigen::Matrix3d& R)
{
  R.setZero();

  double c_y = std::cos(yaw);
  double s_y = std::sin(yaw);

  double c_r = std::cos(roll);
  double s_r = std::sin(roll);

  double c_p = std::cos(pitch);
  double s_p = std::sin(pitch);

  R << c_p*c_y               ,  c_p*s_y                ,  -s_p,
      s_r*s_p*c_y - c_r*s_y ,  s_r*s_p*s_y + c_r*c_y  ,  s_r*c_p,
      c_r*s_p*c_y + s_r*s_y ,  c_r*s_p*s_y - s_r*c_y  ,  c_r*c_p;
}


namespace wolf_navigation
{

// The tracking camera provides us the following transform: odom_T_trackingcamera
// We need to report everything wrt the robot's base (or basefootprint) i.e.: odom_T_base = odom_T_trackingcamera * trackingcamera_T_base
class TrackingCameraEstimator
{

public:

    const std::string CLASS_NAME = "TrackingCameraEstimator";

    /**
   * @brief Shared pointer to TrackingCameraEstimator
   */
    typedef std::shared_ptr<TrackingCameraEstimator> Ptr;

    /**
   * @brief Shared pointer to const TrackingCameraEstimator
   */
    typedef std::shared_ptr<const TrackingCameraEstimator> ConstPtr;

    TrackingCameraEstimator(bool twist_in_local_frame = false);

    ~TrackingCameraEstimator() {}

    // Sets
    void setBaseCameraTransform(const Eigen::Isometry3d& trackingcamera_T_base);
    void setCameraPose(const Eigen::Isometry3d& odom_T_trackingcamera);
    void setCameraTwist(const Eigen::Vector6d &trackingcamera_twist);
    void setCameraLinearTwist(const Eigen::Vector3d& trackingcamera_v);
    void setCameraAngularTwist(const Eigen::Vector3d& trackingcamera_omega);
    void setCameraPoseCovariance(const Eigen::Matrix6d& cov);
    void setCameraTwistCovariance(const Eigen::Matrix6d& cov);

    // Gets
    const Eigen::Isometry3d& getBasePose();
    const Eigen::Vector6d& getBaseTwist();
    Eigen::Vector3d getBaseLinearTwist();
    Eigen::Vector3d getBaseAngularTwist();
    Eigen::Matrix6d getBasePoseCovariance();
    Eigen::Matrix6d getBaseTwistCovariance();
    const Eigen::Vector6d &getBasePose6d();

    void update();


private:

    Eigen::Isometry3d trackingcamera_T_base_; // transform
    Eigen::Isometry3d odom_T_trackingcamera_; // transform
    Eigen::Isometry3d odom_T_base_; // transform
    Eigen::Vector6d trackingcamera_twist_; // twist
    Eigen::Vector6d base_twist_; // twist
    Eigen::Matrix3d base_R_odom_; // rotation matrix
    Eigen::Matrix3d base_R_trackingcamera_; // rotation matrix
    Eigen::Matrix6d base_pose_cov_;
    Eigen::Matrix6d base_twist_cov_;
    Eigen::Matrix6d trackingcamera_pose_cov_;
    Eigen::Matrix6d trackingcamera_twist_cov_;
    Eigen::Vector6d odom_X_base_; // pose6d

    bool twist_in_local_frame_;

};


class BasefootEstimator
{

public:
    BasefootEstimator();

    void setContacts(const std::vector<bool> &foot_contacts, std::vector<double> &foot_height);

    const Eigen::Isometry3d& getBasefootInBase();

    const Eigen::Isometry3d& getBasefootInOdom();

    void setBaseInOdom(const Eigen::Isometry3d& pose);

    void update();

private:
    double estimateHeight();

    std::vector<bool> foot_contacts_;
    std::vector<double> foot_height_;
    Eigen::Isometry3d base_T_basefoot_;
    Eigen::Isometry3d odom_T_basefoot_;
    Eigen::Isometry3d odom_T_base_;
    Eigen::Isometry3d basefoot_T_odom_;

    Eigen::Vector3d tmp_v_;
    Eigen::Matrix3d tmp_R_;

};


} // namespace

#endif
