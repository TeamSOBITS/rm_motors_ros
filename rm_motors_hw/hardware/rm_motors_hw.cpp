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
  gear_ratios_.resize(info_.joints.size(), 1.0);

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

    // Gear Ratio (set automatically based on motor type)
    switch (motor_types_.back()) {
      case rm_motors_can::MotorType::GM6020:
        gear_ratios_[i] = 1.0;
        break;
      case rm_motors_can::MotorType::M3508:
        gear_ratios_[i] = 19.0;
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
  // TODO: Add logic to release the CAN socket if necessary
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
  // On activation, you might want to reset commands to current states
  // For now, we just log and continue.
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsSystemHardware"), "Hardware activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RmMotorsSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsHardware"), "Deactivating hardware...");
  // You might want to send a zero command to all motors here to safely stop them.
  RCLCPP_INFO(rclcpp::get_logger("RmMotorsHardware"), "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RmMotorsSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger("RmMotorsSystemHardware"), "Reading...");
  if (!simulate_)
  {
    rm_motors_can::run_once(gmc_);
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
      // Position reading with unwrapping and gear ratio adjustment (rad)
      double rotor_pos = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Position);
      if (std::isnan(prev_raw_pos_[i])) {
        prev_raw_pos_[i] = rotor_pos;
        unwrapped_rotor_pos_[i] = rotor_pos;
      }
      double delta = rotor_pos - prev_raw_pos_[i];
      while (delta > M_PI) delta -= 2.0 * M_PI;
      while (delta < -M_PI) delta += 2.0 * M_PI;
      unwrapped_rotor_pos_[i] += delta;
      prev_raw_pos_[i] = rotor_pos;
      double output_pos = unwrapped_rotor_pos_[i] / gear_ratios_[i] - position_offsets_[i];
      if (!is_continuous_[i]) {
        // Normalize to [-π, π] for non-continuous joints
        output_pos = std::fmod(output_pos + M_PI, 2.0 * M_PI) - M_PI;
      }
      hw_states_[i][0] = output_pos;
      // Velocity reading (rad/s)
      hw_states_[i][1] = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Velocity) / gear_ratios_[i];
      // Effort (current) reading: convert from motor current (Amps) to torque (Nm)
      hw_states_[i][2] = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Current) * rm_motors_can::nm_per_a(motor_types_[i]);
      // Temperature reading (Celsius)
      hw_states_[i][3] = rm_motors_can::get_state(gmc_, motor_ids_[i], rm_motors_can::FbField::Temperature);
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
      double target_vel = hw_commands_[i];
      double measured_vel = 0.0;

      // Safely fetch measured velocity if available
      if (hw_states_.size() > i && hw_states_[i].size() > 1 && !std::isnan(hw_states_[i][1])) {
        measured_vel = hw_states_[i][1];
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

      // If target velocity is zero, command zero torque and reset PID.
      // Otherwise, calculate torque using the PID controller.
      if (target_vel == 0.0)
      {
        raw_command = 0.0;
        pid_it->second.reset();
      }
      else
      {
        raw_command = pid_it->second.calculate_target_torque(
            target_vel,
            measured_vel,
            period.seconds());
      }
    }
    else if (command_modes_[i] == rm_motors_can::CmdMode::Torque)
    {
      // Effort Mode: Command is already in torque (Nm)
      raw_command = hw_commands_[i];
    }

    if (!simulate_)
    {
      if(rm_motors_can::set_cmd(gmc_, motor_ids_[i], raw_command) < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RmMotorsSystemHardware"), "Error writing command for motor ID %u", motor_ids_[i]);
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  if (!simulate_)
  {
    rm_motors_can::run_once(gmc_);
  }
  return hardware_interface::return_type::OK;
}
} // namespace rm_motors_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rm_motors_hw::RmMotorsSystemHardware, hardware_interface::SystemInterface)
