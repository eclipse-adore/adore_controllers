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

namespace adore
{
namespace controllers
{
void
PID::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "kp_x" )
      kp_x = value;
    if( name == "ki_x" )
      ki_x = value;
    if( name == "velocity_weight" )
      velocity_weight = value;
    if( name == "kp_y" )
      kp_y = value;
    if( name == "ki_y" )
      ki_y = value;
    if( name == "heading_weight" )
      heading_weight = value;
    if( name == "kp_omega" )
      kp_omega = value;
    if( name == "k_feed_forward_ax" )
      k_feed_forward_ax = value;
    if( name == "k_feed_forward_steering_angle" )
      k_feed_forward_steering_angle = value;
    if( name == "dt" && value > 0 )
      dt = value;
    if( name == "steering_comfort" )
      steering_comfort = value;
    if( name == "acceleration_comfort" )
      acceleration_comfort = value;
    if( name == "acceleration_threshold" )
      acceleration_threshold = value;
    if( name == "velocity_threshold" )
      velocity_threshold = value;
    if( name == "constant_brake" )
      constant_brake = value;
    if( name == "lookahead_time" )
      lookahead_time = value;
  }
}

dynamics::VehicleCommand
PID::get_next_vehicle_command( const dynamics::Trajectory& trajectory, const dynamics::VehicleStateDynamic& current_state )
{
  dynamics::VehicleCommand return_command;

  dt_trajectory = trajectory.states[1].time - trajectory.states[0].time;

  // Safety check: Ensure dt is valid and non-zero
  if( dt_trajectory <= std::numeric_limits<double>::epsilon() )
  {
    throw std::runtime_error( "dt trajectory is too small or zero, which may cause division by zero." );
  }

  // Lateral control: lookahead reference
  auto reference_point = trajectory.get_state_at_time( current_state.time + lookahead_time );

  // Precompute sin and cos of the reference point's yaw angle
  double sin_yaw_ref = std::sin( reference_point.yaw_angle );
  double cos_yaw_ref = std::cos( reference_point.yaw_angle );

  // Precompute sin and cos of the current state's yaw angle
  double sin_yaw_current = std::sin( current_state.yaw_angle );
  double cos_yaw_current = std::cos( current_state.yaw_angle );

  // Position error relative to reference
  double delta_x = current_state.x - reference_point.x;
  double delta_y = current_state.y - reference_point.y;

  double error_x = cos_yaw_ref * delta_x + sin_yaw_ref * delta_y; // longitudinal
  double error_y = cos_yaw_ref * delta_y - sin_yaw_ref * delta_x; // lateral

  if( debug_active )
  {
    std::cerr << "Error X (Longitudinal): " << error_x << " m" << std::endl;
    std::cerr << "Error Y (Lateral): " << error_y << " m" << std::endl;
  }

  // Ensure error_y is valid
  if( std::isnan( error_y ) || !std::isfinite( error_y ) )
  {
    error_y = 0.0;
  }

  // Anti-windup: clamp integral term for lateral control
  integral_y = std::clamp( integral_y + error_y * dt, -1.0, 1.0 );

  // Heading error
  double heading_error = std::atan2( -sin_yaw_ref * cos_yaw_current + cos_yaw_ref * sin_yaw_current,
                                     cos_yaw_ref * cos_yaw_current + sin_yaw_ref * sin_yaw_current );

  // Omega error
  double omega_error = current_state.yaw_rate - reference_point.yaw_rate;

  // Steering PID signal
  double pid_signal_psi = k_feed_forward_steering_angle * reference_point.steering_angle - ki_y * integral_y - kp_y * error_y
                        - heading_weight * heading_error - kp_omega * omega_error;

  // Longitudinal control: reference at current time
  reference_point = trajectory.get_state_at_time( current_state.time );

  // Recompute orientation terms
  sin_yaw_ref = std::sin( reference_point.yaw_angle );
  cos_yaw_ref = std::cos( reference_point.yaw_angle );

  sin_yaw_current = std::sin( current_state.yaw_angle );
  cos_yaw_current = std::cos( current_state.yaw_angle );

  // Position error relative to new reference
  delta_x = current_state.x - reference_point.x;
  delta_y = current_state.y - reference_point.y;

  error_x = cos_yaw_ref * delta_x + sin_yaw_ref * delta_y;

  // Ensure error_x is valid
  if( std::isnan( error_x ) || !std::isfinite( error_x ) )
  {
    error_x = 0.0;
  }

  // Anti-windup: clamp integral term for longitudinal control
  integral_x = std::clamp( integral_x + error_x * dt, -1.0, 1.0 );

  // Velocity error
  double error_v = current_state.vx - reference_point.vx;

  if( debug_active )
  {
    std::cerr << "Velocity Error: " << error_v << " m/s" << std::endl;
  }

  if( std::isnan( error_v ) || !std::isfinite( error_v ) )
  {
    error_v = 0.0;
  }

  // Longitudinal PID signal
  double pid_signal_v = k_feed_forward_ax * reference_point.ax - kp_x * error_x - ki_x * integral_x - velocity_weight * error_v;

  // Constant braking when stopped
  if( reference_point.ax < acceleration_threshold && current_state.vx < velocity_threshold )
  {
    pid_signal_v   = constant_brake;
    pid_signal_psi = last_steering_angle;
  }

  // Steering angle velocity control for smoother transitions
  double steering_angle_velocity = ( pid_signal_psi - last_steering_angle ) / dt;

  if( steering_angle_velocity > steering_comfort )
  {
    return_command.steering_angle = last_steering_angle + dt * steering_comfort;
  }
  else if( steering_angle_velocity < -steering_comfort )
  {
    return_command.steering_angle = last_steering_angle - dt * steering_comfort;
  }
  else
  {
    return_command.steering_angle = pid_signal_psi;
  }

  // Longitudinal jerk control for smoother transitions
  double longitudinal_jerk = ( pid_signal_v - last_acceleration ) / dt;

  if( longitudinal_jerk > acceleration_comfort )
  {
    return_command.acceleration = last_acceleration + dt * acceleration_comfort;
  }
  else if( longitudinal_jerk < -acceleration_comfort )
  {
    return_command.acceleration = last_acceleration - dt * acceleration_comfort;
  }
  else
  {
    return_command.acceleration = pid_signal_v;
  }

  // Ensure command values are valid
  if( std::isnan( return_command.acceleration ) || !std::isfinite( return_command.acceleration ) )
  {
    return_command.acceleration = 0.0;
  }

  if( std::isnan( return_command.steering_angle ) || !std::isfinite( return_command.steering_angle ) )
  {
    return_command.steering_angle = 0.0;
  }

  // Update last values for next iteration
  last_steering_angle = return_command.steering_angle;
  last_acceleration   = return_command.acceleration;

  // Enforce model limits
  return_command.clamp_within_limits( model.params );

  return return_command;
}

PID::PID() {} // Initialization

} // namespace controllers
} // namespace adore
