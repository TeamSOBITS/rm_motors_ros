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

  // 3. Derivative Term (D)
  double d_term = 0.0;
  if (delta_time_s > 0.0) {
    d_term = kd_ * (error - previous_error_) / delta_time_s;
  }

  // 4. Integral Term (I): conditional anti-windup, only integrate if the output
  // isn't already saturated in the error's direction (a plain clamp overshoots).
  const double max_i_contribution = max_torque_nm_ / (ki_ > 0.0 ? ki_ : std::numeric_limits<double>::max());
  double integral_candidate = integral_error_;
  if (delta_time_s > 0.0) {
    integral_candidate = std::clamp(
      integral_error_ + error * delta_time_s, -max_i_contribution, max_i_contribution);
  }
  double output_torque_nm = p_term + ki_ * integral_candidate + d_term;
  const bool saturated_further =
    (output_torque_nm >  max_torque_nm_ && error > 0.0) ||
    (output_torque_nm < -max_torque_nm_ && error < 0.0);
  if (!saturated_further) {
    integral_error_ = integral_candidate;
  }
  output_torque_nm = p_term + ki_ * integral_error_ + d_term;

  // 5. Update state
  previous_error_ = error;

  // 6. Clamp the final torque output to the motor's physical limit
  output_torque_nm = std::clamp(output_torque_nm, -max_torque_nm_, max_torque_nm_);

  return output_torque_nm;
}

void RMVelocityPIDController::reset()
{
  integral_error_ = 0.0;
  previous_error_ = 0.0;
}


} // namespace rm_motors_hw
