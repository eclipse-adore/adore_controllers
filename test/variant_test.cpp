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

#include <cmath>
#include <gtest/gtest.h>

#include <map>

#include "controllers/controller.hpp"
#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

using adore::controllers::Controller;
using adore::dynamics::PhysicalVehicleModel;
using adore::dynamics::Trajectory;
using adore::dynamics::VehicleStateDynamic;

TEST( ControllerVariant, EmptyTrajectoryReturnsNullopt )
{
  Controller controller = adore::controllers::PID{};

  Trajectory traj;
  traj.label = "empty";
  traj.states.clear();

  VehicleStateDynamic state{};
  state.time = 0.0;

  PhysicalVehicleModel          model;
  std::map<std::string, double> params; // no overrides for now

  adore::controllers::set_parameters( controller, params, model );

  auto cmd_opt = adore::controllers::get_next_vehicle_command( controller, traj, state );
  EXPECT_FALSE( cmd_opt.has_value() );
}

TEST( ControllerVariant, PidControllerProducesFiniteCommand )
{
  Controller controller = adore::controllers::PID{};

  // Build a simple straight trajectory with two states
  Trajectory traj;
  traj.label = "straight";

  VehicleStateDynamic s0{};
  s0.x              = 0.0;
  s0.y              = 0.0;
  s0.yaw_angle      = 0.0;
  s0.vx             = 1.0;
  s0.ax             = 0.0;
  s0.yaw_rate       = 0.0;
  s0.steering_angle = 0.0;
  s0.time           = 0.0;
  s0.frame_id       = "map";

  VehicleStateDynamic s1 = s0;
  s1.time                = 0.1;

  traj.states.push_back( s0 );
  traj.states.push_back( s1 );

  VehicleStateDynamic current = s0;

  PhysicalVehicleModel          model;
  std::map<std::string, double> params; // you can add PID gains here later

  adore::controllers::set_parameters( controller, params, model );

  auto cmd_opt = adore::controllers::get_next_vehicle_command( controller, traj, current );

  ASSERT_TRUE( cmd_opt.has_value() );
  const auto& cmd = *cmd_opt;

  EXPECT_TRUE( std::isfinite( cmd.acceleration ) );
  EXPECT_TRUE( std::isfinite( cmd.steering_angle ) );
}
