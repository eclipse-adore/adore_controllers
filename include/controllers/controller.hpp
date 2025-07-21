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
#pragma once
#include <optional>
#include <variant>

#include "controllers/PID.hpp"
#include "controllers/iLQR.hpp"
#include "controllers/passthrough.hpp"

namespace adore
{
namespace controllers
{

using Controller = std::variant<PID, iLQR, PassThrough>;

inline void
set_parameters( Controller& controller, const std::map<std::string, double>& controller_settings,
                const dynamics::PhysicalVehicleModel& model )
{
  std::visit( [&]( auto& ctrl ) { // Capture by reference
    ctrl.set_parameters( controller_settings );
    ctrl.model = model;
  }, controller );
}

inline std::optional<dynamics::VehicleCommand>
get_next_vehicle_command( Controller& controller, const dynamics::Trajectory& trajectory,
                          const dynamics::VehicleStateDynamic& vehicle_state )
{
  if( trajectory.states.size() < 2 )
  {
    return std::nullopt;
  }

  // Use std::visit to compute the control action
  dynamics::VehicleCommand controls = std::visit( [&]( auto& ctrl ) { // Capture by reference
    return ctrl.get_next_vehicle_command( trajectory, vehicle_state );
  }, controller );

  return controls;
}


} // namespace controllers
} // namespace adore
