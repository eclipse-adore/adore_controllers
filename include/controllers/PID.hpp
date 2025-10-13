/********************************************************************************
 * Copyright (C) 2017-2023 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Sanath Himasekhar Konthala
 *    Marko Mizdrak
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
  double k_long            = 1.0;
  double k_v               = 1.0;
  double k_feed_forward_ax = 1.0;
  double dt                = 0.1;
  double acc_smoothing     = 0.95;

  double min_lookahead        = 0.1;
  double max_lookahead        = 0.1;
  double base_lookahead       = 0.5;
  double lookahead_gain       = 0.1; // scales with speed
  double slow_steer_smoothing = 4.0;

  // Last known steering angle and acceleration to ensure smooth transitions
  double last_steering_angle = 0.0;
  double last_acceleration   = 0.0;

  bool debug_active = false;


public:

  dynamics::PhysicalVehicleModel model;
  PID();

  // Function to set the PID gains
  void set_parameters( const std::map<std::string, double>& params );

  // Main control function similar to the Stanley controller style
  dynamics::VehicleCommand get_next_vehicle_command( const dynamics::Trajectory&          trajectory,
                                                     const dynamics::VehicleStateDynamic& current_state );

  double compute_steering_angle( const dynamics::VehicleStateDynamic& current_state, const dynamics::VehicleStateDynamic& reference_state );
  double compute_acceleration( const dynamics::VehicleStateDynamic& current_state, const dynamics::VehicleStateDynamic& reference_state );
};

} // namespace controllers
} // namespace adore
