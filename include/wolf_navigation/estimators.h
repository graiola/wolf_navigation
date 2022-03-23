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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

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
    void setCameraLinearTwist(const Eigen::Vector3d& trackingcamera_v);
    void setCameraAngularTwist(const Eigen::Vector3d& trackingcamera_omega);

    // Gets
    const Eigen::Isometry3d& getBasePose();
    const Eigen::Vector3d& getBaseLinearTwist();
    const Eigen::Vector3d& getBaseAngularTwist();

    void update(); // TODO add covariances

private:

    Eigen::Isometry3d trackingcamera_T_base_; // transform
    Eigen::Isometry3d odom_T_trackingcamera_; // transform
    Eigen::Isometry3d odom_T_base_; // transform
    Eigen::Vector3d trackingcamera_v_; // twist linear
    Eigen::Vector3d trackingcamera_omega_; // twist angular
    Eigen::Vector3d base_v_; // twist linear
    Eigen::Vector3d base_omega_; // twist angular
    Eigen::Matrix3d base_R_odom_; // rotation matrix
    Eigen::Matrix3d base_R_trackingcamera_; // rotation matrix

    bool twist_in_local_frame_;

};


} // namespace

#endif
