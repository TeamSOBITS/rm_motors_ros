#include "rm_motors_hw/rm_motors_hw.hpp"
#include "rm_motors_hw/rm_motors_velocity_pid.hpp" // Contains RMVelocityPIDController
#include <rclcpp/rclcpp.hpp>
#include <algorithm> // std::sort, std::adjacent_find
#include <cmath> // For M_PI
#include <vector>
#include <map>
namespace rm_motors_hw
{
hardware_interface::CallbackReturn RmMotorsSystemHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.empty()){
    RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "No joints were specified in the URDF for this hardware interface.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Read hardware parameters ---
  try{
    simulate_ = info_.hardware_parameters.at("simulate")=="true";
    RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Parameter 'simulate' is %s", simulate_ ? "true" : "false");
  }
  catch (const std::out_of_range&){
    RCLCPP_WARN(rclcpp::get_logger("RmMotorsSystemHardware"),"Missing parameter 'simulate'. Assuming false.");
    simulate_ = false;
  }

  if (simulate_)
  {
    can_interface_ = "simulated";
  }
  else
  {
    try{
      can_interface_ = info_.hardware_parameters.at("can_interface");
      RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Got parameter CAN interface: %s", can_interface_.c_str());
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Missing required parameter: 'can_interface'");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // --- Initialize storage vectors ---
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_.resize(info_.joints.size());
  for(std::vector<double>& v : hw_states_)
  {
    v.resize(state_interface_types_.size(), std::numeric_limits<double>::quiet_NaN());
  }
  prev_raw_pos_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  unwrapped_rotor_pos_.resize(info_.joints.size(), 0.0);
  is_continuous_.resize(info_.joints.size());
  invert_rotation_.resize(info_.joints.size());
  gear_ratios_.resize(info_.joints.size(), 1.0);
  feedback_stale_.resize(info_.joints.size(), false);

  size_t i = 0;
  for (const auto & joint : info_.joints)
  {
    // Motor Type
    try{
      std::map<std::string, rm_motors_can::MotorType> type_map{
        {"gm6020", rm_motors_can::MotorType::GM6020},
        {"m3508", rm_motors_can::MotorType::M3508},
        {"m2006", rm_motors_can::MotorType::M2006}};
      motor_types_.emplace_back(type_map.at(joint.parameters.at("motor_type")));
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' missing or incorrect parameter 'motor_type'. Options are \"gm6020\", \"m3508\", \"m2006\".", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Motor ID
    try{
      uint id = std::stoi(joint.parameters.at("motor_id"));
      if (id < 1 || id > 8 || (motor_types_.back() == rm_motors_can::MotorType::GM6020 && id > 7)){
        RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
          "Joint '%s' motor_id out of range [1, %u]: %u", joint.name.c_str(),
          motor_types_.back() == rm_motors_can::MotorType::GM6020 ? 7 : 8, id);
        return hardware_interface::CallbackReturn::ERROR;
      }
      motor_ids_.emplace_back(id);
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' missing required parameter: 'motor_id'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    catch (const std::invalid_argument& e){
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' parameter 'motor_id' is not a number", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    

    // Gear Ratio (set automatically based on motor type)
    switch (motor_types_.back()) {
      case rm_motors_can::MotorType::GM6020:
        gear_ratios_[i] = 1.0;
        break;
      case rm_motors_can::MotorType::M3508:
        gear_ratios_[i] = 3591.0 / 187.0;  // exact M3508 ratio (~19.203), not 19.0
        break;
      case rm_motors_can::MotorType::M2006:
        gear_ratios_[i] = 36.0;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Joint '%s' is a %s, setting gear ratio to %.1f:1", joint.name.c_str(), joint.parameters.at("motor_type").c_str(), gear_ratios_[i]);

    // Command Interface Validation
    if (joint.command_interfaces.size() != 1){
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' has %zu command interfaces. Expected 1.", joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Determine if the joint is continuous based on the command interface
    if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
        is_continuous_[i] = false;
    } else { // velocity or effort
        is_continuous_[i] = true;
    }

    // Position Offset
    try{
      double pos_offset = std::stod(joint.parameters.at("position_offset"));
      if (!is_continuous_[i] && (pos_offset < -M_PI || pos_offset > M_PI)){
        RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
          "Joint '%s' position_offset out of range [-π, π]: %f", joint.name.c_str(), pos_offset);
        return hardware_interface::CallbackReturn::ERROR;
      }
      position_offsets_.emplace_back(pos_offset);
    }
    catch (const std::out_of_range&){
      RCLCPP_WARN(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' missing parameter 'position_offset'. Assuming 0.0", joint.name.c_str());
      position_offsets_.emplace_back(0.0);
    }
    catch (const std::invalid_argument&){
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' parameter 'position_offset' is not a number", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Invert Rotation
    try{
      invert_rotation_[i] = joint.parameters.at("invert_rotation") == "true";
      if (invert_rotation_[i]) {
        RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Joint '%s' has inverted rotation.", joint.name.c_str());
      }
    }
    catch (const std::out_of_range&){
      RCLCPP_WARN(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' missing parameter 'invert_rotation'. Assuming false.", joint.name.c_str());
      invert_rotation_[i] = false;
    }
    std::map<std::string, rm_motors_can::CmdMode> cmd_mode_map {
      {hardware_interface::HW_IF_VELOCITY, rm_motors_can::CmdMode::Velocity},
      {hardware_interface::HW_IF_EFFORT, rm_motors_can::CmdMode::Torque}
    };
    try{
      command_modes_.emplace_back(cmd_mode_map.at(joint.command_interfaces[0].name));
    }
    catch (const std::out_of_range&){
       RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' has an invalid command interface: '%s'. Options are 'velocity' or 'effort'.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // State Interface Validation
    if (joint.state_interfaces.size() != state_interface_types_.size()){
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Joint '%s' has %zu state interfaces. Expected %zu.", joint.name.c_str(),
        joint.state_interfaces.size(), state_interface_types_.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for(size_t j = 0; j < state_interface_types_.size(); j++){
      if (joint.state_interfaces[j].name != state_interface_types_[j]){
        RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
          "Joint '%s' state interface #%zu is '%s', but '%s' was expected.", joint.name.c_str(),
          j, joint.state_interfaces[j].name.c_str(), state_interface_types_[j]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // PID Initialization for Velocity-Commanded Motors
    if (command_modes_.back() == rm_motors_can::CmdMode::Velocity)
    {
      try {
        double kp = std::stod(joint.parameters.at("velocity_kp"));
        double ki = std::stod(joint.parameters.at("velocity_ki"));
        double kd = std::stod(joint.parameters.at("velocity_kd"));

        velocity_pid_controllers_.emplace(i, rm_motors_hw::RMVelocityPIDController(kp, ki, kd,
          rm_motors_can::nm_per_a(motor_types_.back()),
          rm_motors_can::i_max(motor_types_.back())));
        RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"),
          "Joint '%s' (ID: %u, Index: %zu) initialized for Velocity PID Control",
          joint.name.c_str(), motor_ids_.back(), i);
      } catch (const std::out_of_range&) {
        RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
          "Joint '%s' (%s in Velocity Mode) is missing required PID parameters (velocity_kp, velocity_ki, or velocity_kd).",
          joint.name.c_str(), joint.parameters.at("motor_type").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      } catch (const std::invalid_argument&) {
        RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
          "Joint '%s' has a non-numeric PID parameter (velocity_kp/velocity_ki/velocity_kd).",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    i++;
  }

  // Check for duplicate motor IDs
  std::vector<uint> s = motor_ids_;
  std::sort(s.begin(), s.end());
  auto duplicate = std::adjacent_find(s.begin(), s.end());
  if (duplicate != s.end()){
    RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Duplicate motor_id detected: %u. Each joint must have a unique ID.", *duplicate);
      return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RmMotorsSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Configuring hardware...");
  // zero all states and commands
  for(auto& state_vec : hw_states_)
  {
    std::fill(state_vec.begin(), state_vec.end(), 0.0);
  }
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  // Reset unwrap/PID/staleness state so a reconfigure doesn't integrate a stale delta
  std::fill(prev_raw_pos_.begin(), prev_raw_pos_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(unwrapped_rotor_pos_.begin(), unwrapped_rotor_pos_.end(), 0.0);
  std::fill(feedback_stale_.begin(), feedback_stale_.end(), false);
  for (auto & [idx, pid] : velocity_pid_controllers_) { pid.reset(); }
  if (!simulate_)
  {
    if (!(gmc_ = rm_motors_can::init_bus(can_interface_.c_str())))
    {
      RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"),
        "Unable to configure rm_motors CAN driver on interface '%s'", can_interface_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Initialized rm_motors CAN driver on interface '%s'", can_interface_.c_str());
    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
      if (rm_motors_can::init_motor(gmc_, motor_ids_[i], motor_types_[i], command_modes_[i]) < 0)
      {
        RCLCPP_FATAL(rclcpp::get_logger("RmMotorsSystemHardware"), "Unable to initialize motor with ID: %u.", motor_ids_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Initialized motor '%s' (ID: %u) in %s mode.",
          info_.joints[i].name.c_str(), motor_ids_[i], info_.joints[i].command_interfaces[0].name.c_str());
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Hardware configured successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RmMotorsSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsHardware"), "Cleaning up...");
  if (!simulate_ && gmc_ != nullptr)
  {
    // Zero all motor commands and release the CAN socket (period 0 = no ramp)
    rm_motors_can::cleanup(gmc_, 0);
    gmc_ = nullptr;
  }
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsHardware"), "Cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RmMotorsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i][0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i][1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_[i][2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "temperature", &hw_states_[i][3]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RmMotorsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // The command_modes_ were determined in on_init based on the URDF.
    if (command_modes_[i] == rm_motors_can::CmdMode::Velocity)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    else if (command_modes_[i] == rm_motors_can::CmdMode::Torque)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RmMotorsSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Activating hardware...");
  // Don't replay the pre-deactivation command or PID integral on reactivation.
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  for (auto & [idx, pid] : velocity_pid_controllers_) { pid.reset(); }
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Hardware activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RmMotorsSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsHardware"), "Deactivating hardware...");
  // DJI ESCs keep applying the last command, so send an explicit zero here.
  if (!simulate_ && gmc_ != nullptr)
  {
    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
      rm_motors_can::set_cmd(gmc_, motor_ids_[i], 0.0);
    }
    rm_motors_can::run_once(gmc_);
  }
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsHardware"), "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RmMotorsSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  RCLCPP_DEBUG(rclcpp::get_logger("RmMotorsSystemHardware"), "Reading...");
  if (!simulate_)
  {
    // Receive-only: commands are transmitted once per cycle, in write().
    if (rm_motors_can::rx_once(gmc_) < 0)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RmMotorsSystemHardware"), steady_clock_, 1000,
        "CAN rx failed in read(); motor states may be stale");
    }
  }
  for (size_t i = 0; i < hw_states_.size(); i++)
  {
    if (simulate_)
    {
      hw_states_[i][3] = 27.0; // temperature
      hw_states_[i][2] = hw_commands_[i]; // effort
      hw_states_[i][1] = hw_states_[i][1] + (hw_commands_[i] * 0.5 - hw_states_[i][1]) / 2.0; // velocity (simple filter)
      hw_states_[i][0] = hw_states_[i][0] + hw_states_[i][1] * 0.02; // position (assuming 50Hz update)
    }
    else
    {
      // No fresh frames: the PID would integrate a frozen error and slam the motor
      // once the bus recovers, so reset it and have write() command zero torque.
      int64_t fb_age = rm_motors_can::fb_age_ms(gmc_, motor_ids_[i]);
      bool stale = (fb_age < 0) || (fb_age > 150);
      if (stale != feedback_stale_[i])
      {
        if (stale) {
          RCLCPP_WARN(rclcpp::get_logger("RmMotorsSystemHardware"),
            "Motor ID %u feedback is stale (age %ldms) — commanding zero torque", motor_ids_[i], fb_age);
        } else {
          RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"),
            "Motor ID %u feedback recovered", motor_ids_[i]);
        }
        feedback_stale_[i] = stale;
      }
      if (stale)
      {
        auto pid_it = velocity_pid_controllers_.find(i);
        if (pid_it != velocity_pid_controllers_.end()) { pid_it->second.reset(); }
        // Unwrapping across an outage is meaningless — restart from the next fresh sample
        prev_raw_pos_[i] = std::numeric_limits<double>::quiet_NaN();
        continue;
      }

      // Velocity reading first: the rotor speed disambiguates the position unwrap below
      double rotor_vel = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Velocity); // rotor rad/s
      // Position reading with unwrapping and gear ratio adjustment (rad)
      double rotor_pos = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Position);
      // get_state returns NaN on error — keep the previous states for this joint
      if (std::isnan(rotor_pos) || std::isnan(rotor_vel)) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RmMotorsSystemHardware"), steady_clock_, 1000,
          "Invalid feedback for motor ID %u; keeping previous states", motor_ids_[i]);
        prev_raw_pos_[i] = std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      if (std::isnan(prev_raw_pos_[i])) {
        prev_raw_pos_[i] = rotor_pos;
        unwrapped_rotor_pos_[i] = rotor_pos;
      }
      double delta = rotor_pos - prev_raw_pos_[i];
      while (delta > M_PI) delta -= 2.0 * M_PI;
      while (delta < -M_PI) delta += 2.0 * M_PI;
      // ±π unwrap alone tracks < 1500 rotor rpm at 50 Hz; M3508 hits ~9000 rpm,
      // so use the reported velocity to pick the correct 2π multiple.
      double dt = period.seconds();
      if (dt > 0.0 && dt < 1.0) {
        double expected_delta = rotor_vel * dt;
        delta += 2.0 * M_PI * std::round((expected_delta - delta) / (2.0 * M_PI));
      }
      unwrapped_rotor_pos_[i] += delta;
      prev_raw_pos_[i] = rotor_pos;
      // Invert before offset: position_offset is in the joint frame, so it must
      // not flip sign with invert_rotation.
      double output_pos = unwrapped_rotor_pos_[i] / gear_ratios_[i];
      if (invert_rotation_[i]) { output_pos = -output_pos; }
      output_pos -= position_offsets_[i];
      if (!is_continuous_[i]) {
        // Normalize to [-π, π) — fmod is sign-preserving and wrong for negatives
        output_pos = output_pos - 2.0 * M_PI * std::floor((output_pos + M_PI) / (2.0 * M_PI));
      }
      hw_states_[i][0] = output_pos;
      // Velocity state (rad/s at the output shaft)
      double velocity = rotor_vel / gear_ratios_[i];
      hw_states_[i][1] = invert_rotation_[i] ? -velocity : velocity;
      // NaN = field unavailable (e.g. M2006 has no current/temperature feedback).
      double current_a = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Current);
      if (!std::isnan(current_a)) {
        double effort = current_a * rm_motors_can::nm_per_a(motor_types_[i]);
        hw_states_[i][2] = invert_rotation_[i] ? -effort : effort;
      }
      // Temperature reading (Celsius)
      double temperature = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Temperature);
      if (!std::isnan(temperature)) {
        hw_states_[i][3] = temperature;
      }
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("RmMotorsSystemHardware"), "joints read");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmMotorsSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    double raw_command = 0;

    if (command_modes_[i] == rm_motors_can::CmdMode::Velocity)
    {
      // Velocity mode: requires conversion from target velocity (rad/s) to torque (Nm) via PID
      double target_vel = invert_rotation_[i] ? -hw_commands_[i] : hw_commands_[i];
      double measured_vel = 0.0;

      // Safely fetch measured velocity if available
      if (hw_states_.size() > i && hw_states_[i].size() > 1 && !std::isnan(hw_states_[i][1])) {
        measured_vel = invert_rotation_[i] ? -hw_states_[i][1] : hw_states_[i][1];
      } else {
        RCLCPP_DEBUG(rclcpp::get_logger("RmMotorsSystemHardware"),
          "Measured velocity unavailable for joint index %zu; assuming 0.0", i);
      }

      // Ensure a PID controller exists for this joint before calling it
      auto pid_it = velocity_pid_controllers_.find(i);
      if (pid_it == velocity_pid_controllers_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("RmMotorsSystemHardware"),
          "PID controller not initialized for joint index %zu", i);
        return hardware_interface::return_type::ERROR;
      }

      // No feedback → no closed loop: command zero torque (PID was reset in read()).
      if (!simulate_ && feedback_stale_[i])
      {
        raw_command = 0.0;
      }
      else
      {
        // Let the PID regulate to 0 too — the old target==0 special case freewheeled
        // instead of holding zero velocity, and discarded the integral every stop.
        raw_command = pid_it->second.calculate_target_torque(
            target_vel,
            measured_vel,
            period.seconds());
      }
    }
    else if (command_modes_[i] == rm_motors_can::CmdMode::Torque)
    {
      // Effort Mode: Command is already in torque (Nm)
      raw_command = invert_rotation_[i] ? -hw_commands_[i] : hw_commands_[i];
    }

    if (!simulate_)
    {
      if(rm_motors_can::set_cmd(gmc_, motor_ids_[i], raw_command) < 0)
      {
        // Keep going so one failing motor (e.g. overload) doesn't drop the others.
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RmMotorsSystemHardware"), steady_clock_, 1000,
          "Error writing command for motor ID %u (commanding 0)", motor_ids_[i]);
      }
    }
  }
  if (!simulate_)
  {
    if (rm_motors_can::run_once(gmc_) < 0)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RmMotorsSystemHardware"), steady_clock_, 1000,
        "CAN run_once failed in write(); commands may not have been transmitted");
    }
  }
  return hardware_interface::return_type::OK;
}
} // namespace rm_motors_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rm_motors_hw::RmMotorsSystemHardware, hardware_interface::SystemInterface)
