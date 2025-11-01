#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <iostream>

namespace rm_motors_hw
{
/**
 * @brief Implements the velocity control loop (PID) and current command scaling
 * for motors that require a software velocity loop (e.g., M3508, M2006).
 */
class RMVelocityPIDController
{
public:
  /**
   * @brief Construct a new Velocity PID Controller object with tunable PID gains and gear ratio.
   * @param kp Proportional gain for velocity control.
   * @param ki Integral gain for velocity control.
   * @param kd Derivative gain for velocity control.
   */
  RMVelocityPIDController(double kp, double ki, double kd,
                         double nm_per_amp, double max_current_amp);

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

  // PID State Variables
  double integral_error_;
  double previous_error_;
};

} // namespace rm_motors_hw
