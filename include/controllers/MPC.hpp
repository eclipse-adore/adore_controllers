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
#include "multi_agent_solver/multi_agent_solver.hpp"

namespace adore
{
namespace controllers
{

class MPC
{
private:

  struct SolverParams
  {
    double max_iterations = 1000;
    double tolerance      = 1e-3;
    double max_ms         = 50;
    double debug          = 1.0;
  } solver_params;

  double dt = 0.05;

  // Last known steering angle and acceleration to ensure smooth transitions
  double last_steering_angle = 0.0;
  double last_acceleration   = 0.0;

  bool debug_active = false;

  std::shared_ptr<mas::OCP>           problem;
  dynamics::Trajectory                reference_trajectory; // Reference trajectory for the controller
  dynamics::VehicleStateDynamic       start_state;          // Current state of the vehicle
  dynamics::PhysicalVehicleParameters vehicle_params;


public:

  dynamics::PhysicalVehicleModel model;
  MPC();

  // Function to set the PID gains
  void                   set_parameters( const std::map<std::string, double>& params );
  void                   solve_problem();
  void                   setup_problem();
  mas::MotionModel       get_vehicle_model( const dynamics::PhysicalVehicleParameters& params );
  mas::StageCostFunction make_controller_cost( const dynamics::Trajectory& ref_traj );

  // Main control function similar to the Stanley controller style
  dynamics::VehicleCommand get_next_vehicle_command( const dynamics::Trajectory&          trajectory,
                                                     const dynamics::VehicleStateDynamic& current_state );
};

} // namespace controllers
} // namespace adore