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

#include "controllers/MPC.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include <map>

#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

using adore::controllers::MPC;
using adore::dynamics::PhysicalVehicleModel;
using adore::dynamics::Trajectory;
using adore::dynamics::VehicleStateDynamic;

TEST( MPCController, ProducesFiniteCommandOnStraightTrajectory )
{
  MPC                  controller;
  PhysicalVehicleModel model;
  controller.model = model;

  Trajectory traj;
  traj.label = "mpc_straight";

  const double dt    = 0.05; // MPC internal dt
  const double v_ref = 5.0;

  // Build a reference trajectory that covers the full prediction horizon.
  for( int i = 0; i < 50; ++i )
  {
    VehicleStateDynamic state{};
    const double        t = i * dt;

    state.x              = v_ref * t;
    state.y              = 0.0;
    state.yaw_angle      = 0.0;
    state.vx             = v_ref;
    state.ax             = 0.0;
    state.yaw_rate       = 0.0;
    state.steering_angle = 0.0;
    state.time           = t;
    state.frame_id       = "map";

    traj.states.push_back( state );
  }

  VehicleStateDynamic current = traj.states.front();

  std::map<std::string, double> params;
  params["dt"]           = dt;
  params["debug_active"] = 0.0;

  controller.set_parameters( params );

  auto cmd = controller.get_next_vehicle_command( traj, current );

  EXPECT_TRUE( std::isfinite( cmd.acceleration ) );
  EXPECT_TRUE( std::isfinite( cmd.steering_angle ) );
}
