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

#include "controllers/pure_pursuit.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

using adore::controllers::PurePursuit;
using adore::dynamics::PhysicalVehicleModel;
using adore::dynamics::Trajectory;
using adore::dynamics::VehicleStateDynamic;

TEST( PurePursuitController, ProducesFiniteCommandOnStraightTrajectory )
{
  PurePursuit          controller;
  PhysicalVehicleModel model;
  controller.model = model;

  Trajectory traj;
  traj.label = "pure_pursuit_straight";

  const double dt    = 0.1;
  const double v_ref = 5.0;

  // Straight trajectory along x with constant forward velocity.
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

  // Start at the beginning of the trajectory.
  VehicleStateDynamic current = traj.states.front();

  auto cmd = controller.get_next_vehicle_command( traj, current );

  EXPECT_TRUE( std::isfinite( cmd.acceleration ) );
  EXPECT_TRUE( std::isfinite( cmd.steering_angle ) );
}
