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

#include "controllers/NMPC.hpp"

namespace adore
{
namespace controllers
{


void
NMPC::set_parameters( const std::map<std::string, double>& params )
{
  options.intermediateIntegration = 2;
  options.OptiNLC_ACC             = 1e-06;
  options.maxNumberOfIteration    = 500;
  options.OSQP_verbose            = false;
  options.OSQP_max_iter           = 400;
  options.OptiNLC_time_limit      = 0.04;

  for( const auto& [name, value] : params )
  {
    if( name == "k_v" )
      k_v = value;
    if( name == "k_x" )
      k_x = value;
    if( name == "k_y" )
      k_y = value;
    if( name == "k_psi" )
      k_psi = value;
    if( name == "k_delta" )
      k_delta = value;
  }
  options.timeStep = sim_time / control_points;
}

// Helper function to set constraints
void
NMPC::setup_constraints( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp )
{
  // Define a simple input update method
  ocp.setInputUpdate( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& input, double, void* ) {
    VECTOR<double, input_size> update_input = { input[DELTA], input[ACC] };
    return update_input;
  } );

  // State Constraints
  ocp.setUpdateStateLowerBounds( [&]( const VECTOR<double, NMPC::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, NMPC::state_size> state_constraints;
    state_constraints.setConstant( -std::numeric_limits<double>::infinity() );
    return state_constraints;
  } );

  ocp.setUpdateStateUpperBounds( [&]( const VECTOR<double, NMPC::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, NMPC::state_size> state_constraints;
    state_constraints.setConstant( std::numeric_limits<double>::infinity() );
    return state_constraints;
  } );

  // Input Constraints
  ocp.setUpdateInputLowerBounds( [&]( const VECTOR<double, NMPC::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[0] = -limits.max_steering_angle; // Steering angle limit
    input_constraints[1] = limits.min_acceleration;    // Acceleration limit
    return input_constraints;
  } );

  ocp.setUpdateInputUpperBounds( [&]( const VECTOR<double, NMPC::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[0] = limits.max_steering_angle; // Steering angle limit
    input_constraints[1] = limits.max_acceleration;   // Acceleration limit
    return input_constraints;
  } );

  // Define a functions constraints method
  ocp.setUpdateFunctionConstraints( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( 0.0 );
    return functions_constraint;
  } );

  ocp.setUpdateFunctionConstraintsLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( -std::numeric_limits<double>::infinity() );
    return functions_constraint;
  } );

  ocp.setUpdateFunctionConstraintsUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( std::numeric_limits<double>::infinity() );
    return functions_constraint;
  } );
}

void
NMPC::setup_objective_function( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp )
{
  ocp.setObjectiveFunction( [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, input_size>&, double ) {
    return state[L]; // Minimize the cost function `L`
  } );
}

// Helper function to set up the solver and solve the problem
bool
NMPC::solve_mpc( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp,
                 VECTOR<double, state_size>& initial_state, VECTOR<double, input_size>& initial_input, std::vector<double>& delta_output,
                 std::vector<double>& acc_output, double current_time )
{

  OptiNLC_Solver<double, input_size, state_size, 0, control_points> solver( ocp );


  solver.solve( current_time, initial_state, initial_input );

  auto opt_u = solver.get_optimal_inputs();

  for( int i = 0; i < control_points; i++ )
  {
    delta_output.push_back( opt_u[i * input_size + 0] );
    acc_output.push_back( opt_u[i * input_size + 1] );
  }


  return solver.get_final_objective_function() <= 40.0;
}

// Public method to get the next vehicle command based on NMPC
dynamics::VehicleCommand
NMPC::get_next_vehicle_command( const dynamics::Trajectory& trajectory, const dynamics::VehicleStateDynamic& current_state )
{
  dynamics::VehicleCommand return_command;
  auto                     start_time = std::chrono::high_resolution_clock::now();

  // Initial state and input
  VECTOR<double, NMPC::input_size> initial_input = { current_state.steering_angle, current_state.ax };
  VECTOR<double, NMPC::state_size> initial_state = { current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, 0.0 };

  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points> ocp( &options );

  // Set up dynamic model, objective, and constraints
  setup_dynamic_model( ocp, trajectory );

  setup_objective_function( ocp );

  setup_constraints( ocp );

  // Solve the MPC problem
  std::vector<double> delta_output, acc_output;
  bool                success = solve_mpc( ocp, initial_state, initial_input, delta_output, acc_output, current_state.time );

  // return_command.steering_angle = delta_output[0];
  // return_command.acceleration   = acc_output[0];
  double steering_comfort = 200;
  double acceleration_comfort = 205;
  double dt = 0.05;

  double steering_signal = delta_output[0];
  double acc_signal = acc_output[0];

  // Steering angle velocity control for smoother transitions
  double steering_angle_velocity = ( steering_signal - last_steering_angle ) / dt;

  // Clamp steering angle velocity for comfort, ensuring smoother transitions between angles
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
    return_command.steering_angle = steering_signal; // Apply the steering PID signal directly
  }

  // Longitudinal Jerk control for smoother transitions
  double longitudinal_jerk = ( acc_signal - last_acceleration ) / dt;

  // Clamp longitudinal jerk for comfort, ensuring smooth acceleration
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
    return_command.acceleration = acc_signal; // Apply the acceleration PID signal directly
  }

  // Ensure command values are valid and not NaN or infinite
  if( std::isnan( return_command.acceleration ) || !std::isfinite( return_command.acceleration ) )
  {
    return_command.acceleration = 0.0; // Fallback to 0 if NaN or invalid
    // std::cerr << "Acceleration was NaN or invalid, resetting to 0" << std::endl;
  }

  if( std::isnan( return_command.steering_angle ) || !std::isfinite( return_command.steering_angle ) )
  {
    return_command.steering_angle = 0.0; // Fallback to 0 if NaN or invalid
    // std::cerr << "Steering Angle was NaN or invalid, resetting to 0" << std::endl;
  }
  return_command.clamp_within_limits( limits );
  last_steering_angle = return_command.steering_angle;
  last_acceleration = return_command.acceleration;
  std::cerr << "opt out steer: " << steering_signal << "     acc: " << acc_signal << std::endl;

  // Handle bad conditions if the solution is not good
  if( !success )
  {}
  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  // Log cost, time taken, and convergence status
  std::cerr << "NMPC execution time: " << elapsed_seconds.count() << " seconds\n";

  return return_command;
}

void
NMPC::setup_dynamic_model( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp,
                           const dynamics::Trajectory&                                                       trajectory )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, NMPC::input_size>& input,
                            VECTOR<double, NMPC::state_size>& derivative, double current_time, void* ) {
    const double l = 2.69; // wheelbase, can be tuned based on your vehicle

    // Dynamic model equations
    derivative[X]   = state[V] * cos( state[PSI] );       // X derivative (velocity * cos(psi))
    derivative[Y]   = state[V] * sin( state[PSI] );       // Y derivative (velocity * sin(psi))
    derivative[PSI] = state[V] * tan( input[DELTA] ) / l; // PSI derivative (steering angle / wheelbase)
    derivative[V]   = input[ACC];                         // Velocity derivative (acceleration)

    // Interpolate trajectory points based on the current time
    auto reference_point = trajectory.get_state_at_time( current_time );

    // const double lateral_error = -sin( state[PSI] ) * ( state[X] - reference_point.x ) + cos( state[PSI] ) * ( state[Y] -
    // reference_point.y );

    derivative[L] = k_v * ( state[V] - reference_point.vx ) * ( state[V] - reference_point.vx )
                  + k_x * ( state[X] - reference_point.x ) * ( state[X] - reference_point.x )
                  + k_y * ( state[Y] - reference_point.y ) * ( state[Y] - reference_point.y )
                  + k_delta * ( input[DELTA] - last_steering_angle ) * ( input[DELTA] - last_steering_angle )
                  + k_psi * atan2( sin( reference_point.yaw_angle - state[PSI] ), cos( reference_point.yaw_angle - state[PSI] ) )
                      * atan2( sin( reference_point.yaw_angle - state[PSI] ), cos( reference_point.yaw_angle - state[PSI] ) );
  } );
}

NMPC::NMPC()
{
  options.setDefaults();
}


} // namespace controllers
} // namespace adore
