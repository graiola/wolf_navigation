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

    TrackingCameraEstimator() {}

    ~TrackingCameraEstimator() {}

    // Sets
    void setTransform(const Eigen::Isometry3d& transform);
    void setPose(const Eigen::Isometry3d& pose);
    void setLinearTwist(const Eigen::Vector3d& linear);
    void setAngularTwist(const Eigen::Vector3d& angular);

    // Gets
    const Eigen::Isometry3d& getPose();
    const Eigen::Vector3d& getLinearTwist();
    const Eigen::Vector3d& getAngularTwist();

    void update(bool twist_in_local_frame = false); // TODO add covariances

private:

    Eigen::Isometry3d trackingcamera_T_basefootprint_; // transform
    Eigen::Isometry3d odom_T_trackingcamera_; // transform
    Eigen::Isometry3d odom_T_basefootprint_; // transform
    Eigen::Vector3d trackingcamera_v_; // twist linear
    Eigen::Vector3d trackingcamera_omega_; // twist angular
    Eigen::Vector3d basefootprint_v_; // twist linear
    Eigen::Vector3d basefootprint_omega_; // twist angular
    Eigen::Matrix3d basefootprint_R_odom_; // rotation matrix
    Eigen::Matrix3d basefootprint_R_trackingcamera_; // rotation matrix

};


} // namespace

#endif
