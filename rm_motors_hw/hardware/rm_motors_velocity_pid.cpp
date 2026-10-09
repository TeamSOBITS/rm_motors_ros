#include "rm_motors_hw/rm_motors_velocity_pid.hpp"

namespace rm_motors_hw
{

// --- RMVelocityPIDController Implementation ---

RMVelocityPIDController::RMVelocityPIDController(double kp, double ki, double kd,
                                                 double nm_per_amp, double max_current_amp,
                                                 const Config & config)
  : kp_(kp), ki_(ki), kd_(kd),
    nm_per_amp_(nm_per_amp), max_torque_nm_(nm_per_amp * max_current_amp),
    i_decay_tau_s_(config.i_decay_tau_s), vel_lpf_hz_(config.vel_lpf_hz), i_limit_nm_(config.i_limit_nm),
    integral_error_(0.0),
    filtered_vel_(0.0), vel_filter_initialized_(false)
{
}

double RMVelocityPIDController::calculate_target_torque(
  double target_vel_rad_s,
  double measured_vel_rad_s,
  double delta_time_s)
{
  // 1. Low-pass filter the measured velocity (P/I/D all act on the filtered value).
  const bool first_sample = !vel_filter_initialized_;
  const double prev_filtered_vel = filtered_vel_;
  if (first_sample) {
    filtered_vel_ = measured_vel_rad_s;
    vel_filter_initialized_ = true;
  } else if (vel_lpf_hz_ > 0.0 && delta_time_s > 0.0) {
    const double tau = 1.0 / (2.0 * M_PI * vel_lpf_hz_);
    const double alpha = delta_time_s / (tau + delta_time_s);
    filtered_vel_ += alpha * (measured_vel_rad_s - filtered_vel_);
  } else if (vel_lpf_hz_ <= 0.0) {
    filtered_vel_ = measured_vel_rad_s;
  }

  // 2. Calculate Error
  double error = target_vel_rad_s - filtered_vel_;

  // 3. Proportional Term (P)
  double p_term = kp_ * error;

  // 4. Derivative Term (D): on filtered measurement, not error, so setpoint steps
  // don't kick. Skipped on the first sample after reset and when dt<=0.
  double d_term = 0.0;
  if (!first_sample && delta_time_s > 0.0) {
    d_term = -kd_ * (filtered_vel_ - prev_filtered_vel) / delta_time_s;
  }

  // 5. Integral Term (I): conditional anti-windup, only integrate if the output
  // isn't already saturated in the error's direction (a plain clamp overshoots).
  const double i_bound_nm = i_limit_nm_ > 0.0 ? i_limit_nm_ : max_torque_nm_;
  const double max_i_contribution = i_bound_nm / (ki_ > 0.0 ? ki_ : std::numeric_limits<double>::max());
  double integral_candidate = integral_error_;
  // Near-zero target: bleed the integral down instead of integrating, so a locked
  // wheel (error == 0 at standstill) doesn't hold residual torque forever.
  if (std::abs(target_vel_rad_s) < 1e-3 && i_decay_tau_s_ > 0.0) {
    if (delta_time_s > 0.0) {
      integral_candidate = integral_error_ * std::exp(-delta_time_s / i_decay_tau_s_);
    }
  } else if (delta_time_s > 0.0) {
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

  // 6. Clamp the final torque output to the motor's physical limit
  output_torque_nm = std::clamp(output_torque_nm, -max_torque_nm_, max_torque_nm_);

  return output_torque_nm;
}

void RMVelocityPIDController::reset()
{
  integral_error_ = 0.0;
  vel_filter_initialized_ = false;
}


} // namespace rm_motors_hw
