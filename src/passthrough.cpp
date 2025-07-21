/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
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
