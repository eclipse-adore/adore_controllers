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

#include "adore_math/fast_trig.h"

namespace adore
{
namespace controllers
{
void
MPC::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "dt" && value > 0 ) // Ensure dt > 0
      dt = value;
    if( name == "horizon_steps" && value > 0 )
      horizon_steps = static_cast<size_t>( value );
    // weights
    if( name == "londitudinal_weight" )
      weights.longitudinal = value;
    if( name == "heading_weight" )
      weights.heading = value;
    if( name == "vel_weight" )
      weights.vel = value;
    if( name == "steering_weight" )
      weights.steering = value;
    if( name == "acceleration_weight" )
      weights.acceleration = value;
    if( name == "continuity_accel_weight" )
      weights.continuity_accel = value;
    if( name == "continuity_steering_weight" )
      weights.continuity_steering = value;
    // solver params
    if( name == "max_iterations" )
      solver_params.max_iterations = value;
    if( name == "tolerance" )
      solver_params.tolerance = value;
    if( name == "max_ms" )
      solver_params.max_ms = value;
    else if( name == "debug_active" )
      debug_active = static_cast<bool>( value );
  }
}

dynamics::VehicleCommand
MPC::get_next_vehicle_command( const dynamics::Trajectory& trajectory, const dynamics::VehicleStateDynamic& current_state )
{
  dynamics::VehicleCommand command;

  start_state                = current_state;
  reference_trajectory       = trajectory;
  double state_trajectory_dt = trajectory.states[0].time - current_state.time;
  reference_trajectory.adjust_start_time( state_trajectory_dt );
  setup_problem();
  solve_problem();

  const auto& s0 = previous_traj.states[0];
  const auto& s1 = previous_traj.states[1];

  command.steering_angle = 0.5 * ( s0.steering_angle + s1.steering_angle );
  command.acceleration   = 0.5 * ( s0.ax + s1.ax );

  command.clamp_within_limits( model.params );
  last_command = command;
  return command;
}

mas::StageCostFunction
MPC::make_controller_cost( const dynamics::Trajectory& ref_traj )
{
  return [weights = weights, start_state = start_state, ref_traj = ref_traj, dt = dt]( const mas::State& x, const mas::Control& u,
                                                                                       std::size_t k ) -> double {
    double cost = 0.0;

    const double t   = k * dt;
    const auto   ref = ref_traj.get_state_at_time( t );

    const double dx = x( 0 ) - ref.x;
    const double dy = x( 1 ) - ref.y;

    const double c = math::fast_cos( ref.yaw_angle );
    const double s = math::fast_sin( ref.yaw_angle );

    const double lon_err = dx * c + dy * s;
    const double lat_err = -dx * s + dy * c;
    const double hdg_err = math::normalize_angle( x( 2 ) - ref.yaw_angle );
    const double spd_err = x( 3 ) - ref.vx;

    cost += weights.longitudinal * lon_err * lon_err;
    cost += weights.lateral * lat_err * lat_err;
    cost += weights.heading * hdg_err * hdg_err;
    cost += weights.vel * spd_err * spd_err;
    cost += weights.steering * u( 0 ) * u( 0 );
    cost += weights.acceleration * u( 1 ) * u( 1 );
    // continuity costs
    if( k == 0 )
    {
      const double d_steering  = u( 0 ) - start_state.steering_angle;
      const double d_accel     = u( 1 ) - start_state.ax;
      cost                    += weights.continuity_steering * d_steering * d_steering;
      cost                    += weights.continuity_accel * d_accel * d_accel;
    }
    return cost;
  };
}

mas::MotionModel
MPC::get_vehicle_model( const dynamics::PhysicalVehicleParameters& params )
{
  return [params]( const mas::State& x, const mas::Control& u ) -> mas::StateDerivative {
    mas::StateDerivative dxdt;
    dxdt.setZero( 4 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );                    // x
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );                    // y
    dxdt( 2 ) = x( 3 ) * std::tan( u( 0 ) ) / params.wheelbase; // yaw_angle
    dxdt( 3 ) = u( 1 );                                         // v
    return dxdt;
  };
}

void
MPC::setup_problem()
{
  problem = std::make_shared<mas::OCP>();

  problem->state_dim     = 4;
  problem->control_dim   = 2;
  problem->horizon_steps = horizon_steps;
  problem->dt            = dt;
  problem->initial_state = Eigen::VectorXd( 4 );
  problem->dynamics      = get_vehicle_model( vehicle_params );

  Eigen::VectorXd lower_bounds( problem->control_dim ), upper_bounds( problem->control_dim );
  lower_bounds << -vehicle_params.steering_angle_max, vehicle_params.acceleration_min;
  upper_bounds << vehicle_params.steering_angle_max, vehicle_params.acceleration_max;
  problem->input_lower_bounds = lower_bounds;
  problem->input_upper_bounds = upper_bounds;
  problem->stage_cost         = make_controller_cost( reference_trajectory );
  problem->terminal_cost      = []( const mas::State& x ) -> double { return 0.0; };

  // set reference trajectory as initial control guess
  problem->initial_controls = mas::ControlTrajectory::Zero( problem->control_dim, problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    const double t                    = i * problem->dt;
    const auto   ref                  = reference_trajectory.get_state_at_time( t );
    problem->initial_controls( 1, i ) = ref.ax;
    problem->initial_controls( 0, i ) = ref.steering_angle;
  }

  problem->initial_state << start_state.x, start_state.y, start_state.yaw_angle, start_state.vx;
  problem->initialize_problem();
  problem->verify_problem();
}

void
MPC::solve_problem()
{
  mas::SolverParams params;
  params["max_iterations"] = solver_params.max_iterations;
  params["tolerance"]      = solver_params.tolerance;
  params["max_ms"]         = solver_params.max_ms;
  params["debug"]          = solver_params.debug;


  auto solve_with = [&]( auto&& solver, double max_ms ) {
    params["max_ms"] = max_ms;
    solver.set_params( params );
    solver.solve( *problem );
  };
  solve_with( mas::OSQP{}, solver_params.max_ms );
  previous_traj = extract_trajectory();
}

dynamics::Trajectory
controllers::MPC::get_last_trajectory() const // for visualizing
{
  return previous_traj;
}

dynamics::Trajectory
MPC::extract_trajectory()
{
  dynamics::Trajectory trajectory;
  trajectory.states.reserve( problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    dynamics::VehicleStateDynamic state;
    auto                          x = problem->best_states.col( i );
    auto                          u = problem->best_controls.col( i );

    state.x              = x( 0 );
    state.y              = x( 1 );
    state.yaw_angle      = math::normalize_angle( x( 2 ) );
    state.vx             = x( 3 );
    state.time           = start_state.time + i * dt;
    state.steering_angle = math::normalize_angle( u( 0 ) );
    state.ax             = u( 1 );

    trajectory.states.push_back( state );
  }

  return trajectory;
}

MPC::MPC() {} // Initialization
} // namespace controllers
} // namespace adore