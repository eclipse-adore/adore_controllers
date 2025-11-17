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

#include "controllers/PID.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include <stdexcept>

#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

using adore::controllers::PID;
using adore::dynamics::PhysicalVehicleModel;
using adore::dynamics::Trajectory;
using adore::dynamics::VehicleStateDynamic;

TEST( PIDController, ThrowsOnZeroDtTrajectory )
{
  PID                  pid;
  PhysicalVehicleModel model;
  pid.model = model;

  Trajectory          traj;
  VehicleStateDynamic s0{};
  s0.time = 0.0;

  VehicleStateDynamic s1{};
  s1.time = 0.0; // dt = 0

  traj.states.push_back( s0 );
  traj.states.push_back( s1 );

  VehicleStateDynamic current{};
  current.time = 0.0;

  EXPECT_THROW( pid.get_next_vehicle_command( traj, current ), std::runtime_error );
}

TEST( PIDController, ProducesFiniteCommandOnStraightTrajectory )
{
  PID                  pid;
  PhysicalVehicleModel model;
  pid.model = model;

  Trajectory traj;
  traj.label = "straight";

  VehicleStateDynamic s0{};
  s0.x              = 0.0;
  s0.y              = 0.0;
  s0.yaw_angle      = 0.0;
  s0.vx             = 0.0;
  s0.ax             = 0.0;
  s0.yaw_rate       = 0.0;
  s0.steering_angle = 0.0;
  s0.time           = 0.0;
  s0.frame_id       = "map";

  VehicleStateDynamic s1 = s0;
  s1.time                = 0.1; // dt_trajectory > 0

  traj.states.push_back( s0 );
  traj.states.push_back( s1 );

  VehicleStateDynamic current = s0;

  auto cmd = pid.get_next_vehicle_command( traj, current );

  EXPECT_TRUE( std::isfinite( cmd.acceleration ) );
  EXPECT_TRUE( std::isfinite( cmd.steering_angle ) );

  // Sanity bounds (we don't care about exact values here)
  EXPECT_LT( std::abs( cmd.acceleration ), 100.0 );
  EXPECT_LT( std::abs( cmd.steering_angle ), 2.0 );
}
