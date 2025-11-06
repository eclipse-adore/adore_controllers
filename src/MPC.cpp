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
    if( name == "debug_active" )
      debug_active = static_cast<bool>( value );
  }
}

dynamics::VehicleCommand
MPC::get_next_vehicle_command( const dynamics::Trajectory& trajectory, const dynamics::VehicleStateDynamic& current_state )
{
  dynamics::VehicleCommand command;
  std::cerr << "using mpc controller" << std::endl;

  start_state          = current_state;
  reference_trajectory = trajectory;
  setup_problem();
  solve_problem();

  auto u                 = problem->best_controls.col( 0 );
  command.steering_angle = u( 0 );
  command.acceleration   = u( 1 );

  last_acceleration   = command.acceleration;
  last_steering_angle = command.steering_angle;
  command.clamp_within_limits( model.params );
  return command;
}

mas::StageCostFunction
MPC::make_controller_cost( const dynamics::Trajectory& ref_traj )
{
  return [start_state = start_state, ref_traj = ref_traj, dt = dt]( const mas::State& x, const mas::Control& u, std::size_t k ) -> double {
    double cost = 0.0;

    const double t   = k * dt;
    const auto   ref = ref_traj.get_state_at_time( t );

    const double dx = x( 0 ) - ref.x;
    const double dy = x( 1 ) - ref.y;

    const double x_err   = dx;
    const double y_err   = dy;
    const double hdg_err = 0.5 * atan2( sin( x( 2 ) - ref.yaw_angle ), cos( x( 2 ) - ref.yaw_angle ) )
                         * atan2( sin( x( 2 ) - ref.yaw_angle ), cos( x( 2 ) - ref.yaw_angle ) );
    const double spd_err = x( 3 ) - ref.vx;

    cost += 0.5 * x_err * x_err;
    cost += 0.5 * y_err * y_err;
    cost += 0.5 * hdg_err * hdg_err;
    cost += 0.5 * spd_err * spd_err;
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
  problem->horizon_steps = 20;
  problem->dt            = 0.05;
  problem->initial_state = Eigen::VectorXd( 4 );
  problem->dynamics      = get_vehicle_model( vehicle_params );

  Eigen::VectorXd lower_bounds( problem->control_dim ), upper_bounds( problem->control_dim );
  lower_bounds << -vehicle_params.steering_angle_max, vehicle_params.acceleration_min;
  upper_bounds << vehicle_params.steering_angle_max, vehicle_params.acceleration_max;
  problem->input_lower_bounds = lower_bounds;
  problem->input_upper_bounds = upper_bounds;
  problem->stage_cost         = make_controller_cost( reference_trajectory );
  problem->terminal_cost      = []( const mas::State& x ) -> double { return 0.0; };

  problem->initial_state << start_state.x, start_state.y, start_state.yaw_angle, start_state.vx;
  problem->initialize_problem();
  problem->verify_problem();
}

void
MPC::solve_problem() // THIS ONE WORKS
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
}

MPC::MPC() {} // Initialization
} // namespace controllers
} // namespace adore