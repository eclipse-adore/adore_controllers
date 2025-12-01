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

#include "controllers/passthrough.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

using adore::controllers::PassThrough;
using adore::dynamics::PhysicalVehicleModel;
using adore::dynamics::Trajectory;
using adore::dynamics::VehicleStateDynamic;

TEST( PassThroughController, MirrorsFirstTrajectoryState )
{
  PassThrough          controller;
  PhysicalVehicleModel model;
  controller.model = model;

  Trajectory traj;
  traj.label = "passthrough_traj";

  VehicleStateDynamic s0{};
  s0.x              = 0.0;
  s0.y              = 0.0;
  s0.yaw_angle      = 0.0;
  s0.vx             = 3.0;
  s0.ax             = 1.2;
  s0.yaw_rate       = 0.0;
  s0.steering_angle = 0.15;
  s0.time           = 0.0;
  s0.frame_id       = "map";

  VehicleStateDynamic s1 = s0;
  s1.time                = 0.1;

  traj.states.push_back( s0 );
  traj.states.push_back( s1 );

  VehicleStateDynamic current = s0;

  auto cmd = controller.get_next_vehicle_command( traj, current );

  EXPECT_TRUE( std::isfinite( cmd.acceleration ) );
  EXPECT_TRUE( std::isfinite( cmd.steering_angle ) );

  // PassThrough should simply forward the reference values from the first state.
  EXPECT_DOUBLE_EQ( cmd.acceleration, s0.ax );
  EXPECT_DOUBLE_EQ( cmd.steering_angle, s0.steering_angle );
}
