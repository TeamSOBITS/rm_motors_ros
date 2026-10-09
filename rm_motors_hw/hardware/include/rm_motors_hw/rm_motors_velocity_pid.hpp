#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <iostream>

namespace rm_motors_hw
{
/// Optional PID tuning; the defaults reproduce the original PID exactly.
struct RMVelocityPIDConfig
{
  double i_decay_tau_s = 0.0;  // <=0 disables the zero-target integral bleed
  double vel_lpf_hz = 0.0;     // <=0 disables the velocity low-pass filter
  double i_limit_nm = 0.0;     // <=0 bounds the integral by max_torque_nm_ instead
};

/**
 * @brief Implements the velocity control loop (PID) and current command scaling
 * for motors that require a software velocity loop (e.g., M3508, M2006).
 */
class RMVelocityPIDController
{
public:
  using Config = RMVelocityPIDConfig;

  /**
   * @brief Construct a new Velocity PID Controller object with tunable PID gains and gear ratio.
   * @param kp Proportional gain for velocity control.
   * @param ki Integral gain for velocity control.
   * @param kd Derivative gain for velocity control.
   */
  RMVelocityPIDController(double kp, double ki, double kd,
                         double nm_per_amp, double max_current_amp,
                         const Config & config = Config());

  /**
   * @brief Core control function: converts desired velocity to required current.
   * @param target_vel_rad_s The desired joint/wheel angular velocity (rad/s) 
   * @param measured_vel_rad_s The actual joint/wheel angular velocity (rad/s).
   * @param delta_time_s The time elapsed since the last control update (seconds).
   * @return double The resulting required torque in Nm.
   */
  double calculate_target_torque(
    double target_vel_rad_s,
    double measured_vel_rad_s,
    double delta_time_s);

  /**
   * @brief Resets the internal state of the PID controller (integral and derivative terms).
   *
   */
  void reset();

public:
  // PID Gains
  double kp_;
  double ki_;
  double kd_;

  // Motor Physical Constants
  double nm_per_amp_;     // Kt (Nm/A)
  double max_torque_nm_;  // I_max * Kt (Nm)

  // Optional tuning (see Config)
  double i_decay_tau_s_;
  double vel_lpf_hz_;
  double i_limit_nm_;

  // PID State Variables
  double integral_error_;

  // Low-pass-filtered measured velocity; also serves as the derivative-on-measurement
  // history, so it doubles as the "first sample since reset" flag for both.
  double filtered_vel_;
  bool vel_filter_initialized_;
};

} // namespace rm_motors_hw
