#include "rm_motors_hw/rm_motors_velocity_pid.hpp"

namespace rm_motors_hw
{

// --- RMVelocityPIDController Implementation ---

RMVelocityPIDController::RMVelocityPIDController(double kp, double ki, double kd,
                                                 double nm_per_amp, double max_current_amp)
  : kp_(kp), ki_(ki), kd_(kd),
    nm_per_amp_(nm_per_amp), max_torque_nm_(nm_per_amp * max_current_amp),
    integral_error_(0.0), previous_error_(0.0)
{
}

double RMVelocityPIDController::calculate_target_torque(
  double target_vel_rad_s,
  double measured_vel_rad_s,
  double delta_time_s)
{
  // 1. Calculate Error
  double error = target_vel_rad_s - measured_vel_rad_s;

  // 2. Proportional Term (P)
  double p_term = kp_ * error;

  // 3. Integral Term (I)
  if (delta_time_s > 0.0) {
    integral_error_ += error * delta_time_s;
  }

  // Anti-windup: Clamp the integral term based on maximum possible torque (I_max * Kt)
  // Max I contribution: Max Torque / Ki
  const double max_i_contribution = max_torque_nm_ / (ki_ > 0.0 ? ki_ : std::numeric_limits<double>::max());
  integral_error_ = std::clamp(integral_error_, -max_i_contribution, max_i_contribution);
  
  double i_term = ki_ * integral_error_;

  // 4. Derivative Term (D)
  double d_term = 0.0;
  if (delta_time_s > 0.0) {
    d_term = kd_ * (error - previous_error_) / delta_time_s;
  }

  // 5. Calculate Total Output (Target Torque in Nm)
  double output_torque_nm = p_term + i_term + d_term;

  // 6. Update state
  previous_error_ = error;

  // 7. Clamp the final torque output to the motor's physical limit
  output_torque_nm = std::clamp(output_torque_nm, -max_torque_nm_, max_torque_nm_);

  return output_torque_nm;
}

} // namespace rm_motors_hw
