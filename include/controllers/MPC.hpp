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
    double max_ms         = 20;
    double debug          = 1.0;
  } solver_params;

  struct Weights
  {
    double longitudinal        = 1.0;
    double lateral             = 10.0;
    double heading             = 10.0;
    double vel                 = 10.0;
    double steering            = 0.1;
    double acceleration        = 0.01;
    double continuity_accel    = 0.1;
    double continuity_steering = 0.1;
  } weights;

  double dt            = 0.1;
  size_t horizon_steps = 10;

  // Last known steering angle and acceleration to ensure smooth transitions
  dynamics::VehicleCommand last_command;

  bool debug_active = false;

  std::shared_ptr<mas::OCP>           problem;
  dynamics::Trajectory                reference_trajectory; // Reference trajectory for the controller
  dynamics::VehicleStateDynamic       start_state;          // Current state of the vehicle
  dynamics::PhysicalVehicleParameters vehicle_params;
  dynamics::Trajectory                previous_traj;


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
  dynamics::Trajectory     get_last_trajectory() const;

  dynamics::Trajectory extract_trajectory();
};

} // namespace controllers
} // namespace adore