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

#include <memory>
#include <Eigen/Core>

#include <nav_msgs/Odometry.h>

namespace wolf_navigation
{

template <typename ...>
class EstimatorInterface
{

public:

  const std::string CLASS_NAME = "EstimatorInterface";

  /**
   * @brief Shared pointer to Estimator
   */
  typedef std::shared_ptr<EstimatorInterface> Ptr;

  /**
   * @brief Shared pointer to const Estimator
   */
  typedef std::shared_ptr<const EstimatorInterface> ConstPtr;


  EstimatorInterface();

  ~EstimatorInterface() {}




private:


};


} // namespace

#endif
