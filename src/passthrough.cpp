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

namespace adore
{
namespace controllers
{
void
PassThrough::set_parameters( const std::map<std::string, double>& params )
{
  // no params
}

dynamics::VehicleCommand
PassThrough::get_next_vehicle_command( const dynamics::Trajectory& trajectory, const dynamics::VehicleStateDynamic& current_state )
{
  dynamics::VehicleCommand command;
  command.acceleration   = trajectory.states[0].ax;
  command.steering_angle = trajectory.states[0].steering_angle;
  return command;
}

PassThrough::PassThrough() {} // Initialization
} // namespace controllers
} // namespace adore
