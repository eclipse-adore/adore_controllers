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
 *    Sanath Himasekhar Konthala
 *    Marko Mizdrak
 ********************************************************************************/
#include "controllers/PID.hpp"

namespace adore
{
namespace controllers
{
void
PID::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "acc_smoothing" )
      acc_smoothing = value;
    if( name == "k_long" )
      k_long = value;
    if( name == "k_v" )
      k_v = value;
    if( name == "k_feed_forward_ax" )
      k_feed_forward_ax = value;
    if( name == "dt" && value > 0 ) // Ensure dt > 0
      dt = value;
    if( name == "min_lookahead" )
      min_lookahead = value;
    if( name == "max_lookahead" )
      max_lookahead = value;
    if( name == "base_lookahead" )
      base_lookahead = value;
    if( name == "lookahead_gain" )
      lookahead_gain = value;
    if( name == "slow_steer_smoothing" )
      slow_steer_smoothing = value;
    if( name == "debug_active" )
      debug_active = static_cast<bool>( value );
  }
}

dynamics::VehicleCommand
PID::get_next_vehicle_command( const dynamics::Trajectory& trajectory, const dynamics::VehicleStateDynamic& current_state )
{
  dynamics::VehicleCommand command;

  const double vx                 = current_state.vx;
  const double steering_lookahead = std::clamp( base_lookahead + lookahead_gain * vx, min_lookahead, max_lookahead );

  auto nearest_state = trajectory.get_nearest_state( current_state );

  double ref_time = nearest_state.time;

  const auto steer_reference = trajectory.get_state_at_time( ref_time + steering_lookahead );
  const auto acc_reference   = trajectory.get_state_at_time( current_state.time );

  command.acceleration   = compute_acceleration( current_state, acc_reference );
  command.steering_angle = compute_steering_angle( current_state, steer_reference );

  last_acceleration   = command.acceleration;
  last_steering_angle = command.steering_angle;
  command.clamp_within_limits( model.params );
  return command;
}

double
PID::compute_steering_angle( const dynamics::VehicleStateDynamic& current_state, const dynamics::VehicleStateDynamic& reference_state )
{
  const double sin_yaw_ref = std::sin( reference_state.yaw_angle );
  const double cos_yaw_ref = std::cos( reference_state.yaw_angle );

  const double dx = current_state.x - reference_state.x;
  const double dy = current_state.y - reference_state.y; // CHANGED

  double error_y = -sin_yaw_ref * dx + cos_yaw_ref * dy;
  double error_x = cos_yaw_ref * dx + sin_yaw_ref * dy;

  double alpha = std::atan2( -dy, -dx ) - current_state.yaw_angle;
  alpha        = math::normalize_angle( alpha );

  // Pure‑Pursuit steering law inspired by autoware version
  double steer_signal = std::atan2( 2.0 * model.params.wheelbase * std::sin( alpha ), -error_x )
                      * std::tanh( -error_x * slow_steer_smoothing );
  return steer_signal;
}

double
PID::compute_acceleration( const dynamics::VehicleStateDynamic& current_state, const dynamics::VehicleStateDynamic& reference_state )
{

  const double sin_yaw_ref = std::sin( reference_state.yaw_angle );
  const double cos_yaw_ref = std::cos( reference_state.yaw_angle );

  const double dx = current_state.x - reference_state.x;
  const double dy = current_state.y - reference_state.y; // CHANGED

  double error_x = cos_yaw_ref * dx + sin_yaw_ref * dy;
  double error_v = current_state.vx - reference_state.vx;

  double acc_signal = k_feed_forward_ax * reference_state.ax - k_long * error_x - k_v * error_v;
  acc_signal        = ( 1 - acc_smoothing ) * acc_signal + acc_smoothing * last_acceleration;

  return acc_signal;
}

PID::PID() {} // Initialization
} // namespace controllers
} // namespace adore
