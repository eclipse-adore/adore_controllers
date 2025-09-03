/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include <cmath>

#include <chrono>
#include <iostream>

#include "adore_math/angles.h"
#include "adore_math/distance.h"

#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/trajectory.hpp"

namespace adore
{
namespace controllers
{

class PID
{
private:

  // PID Gains
  double kp_x                          = 0.0;
  double ki_x                          = 0.0;
  double velocity_weight               = 5.0;
  double kp_y                          = 5.0;
  double ki_y                          = 0.0;
  double heading_weight                = 5.0;
  double kp_omega                      = 0.5;
  double k_feed_forward_ax             = 1.0;
  double k_feed_forward_steering_angle = 1.0;
  double dt                            = 0.05;
  double steering_comfort              = 2.5; // Comfort limit for steering change
  double acceleration_comfort          = 2.5; // Comfort limit for acceleration change
  double lookahead_time                = 0.5;
  double dt_trajectory                 = 0.1; // dt between points in the trajectory

  // State variables for integral control
  double integral_x = 0.0;
  double integral_y = 0.0;

  // Last known steering angle and acceleration to ensure smooth transitions
  double last_steering_angle = 0.0;
  double last_acceleration   = 0.0;

  // Parameters for stopping at standstill
  double acceleration_threshold = 0.05;
  double velocity_threshold = 1.0;
  double constant_brake = 0.5;

  bool debug_active = false;


public:

  dynamics::PhysicalVehicleModel model;


  dynamics::VehicleCommandLimits limits;
  PID();

  // Function to set the PID gains
  void set_parameters( const std::map<std::string, double>& params );

  // Main control function similar to the Stanley controller style
  dynamics::VehicleCommand get_next_vehicle_command( const dynamics::Trajectory&          trajectory,
                                                     const dynamics::VehicleStateDynamic& current_state );
};

} // namespace controllers
} // namespace adore
