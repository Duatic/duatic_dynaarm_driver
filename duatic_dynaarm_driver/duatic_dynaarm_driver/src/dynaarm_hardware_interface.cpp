/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "duatic_dynaarm_driver/dynaarm_hardware_interface.hpp"
#include "duatic_dynaarm_driver/kinematic_translation.hpp"

using namespace duatic_ros2control_hardware;  // NOLINT(build/namespaces)

namespace duatic_dynaarm_driver
{

DynaArmHardwareInterface::DynaArmHardwareInterface() : logger_(rclcpp::get_logger("DynaArmHardwareInterface"))
{
}

std::vector<hardware_interface::InterfaceDescription>
DynaArmHardwareInterface::export_unlisted_state_interface_descriptions()
{
}
std::vector<hardware_interface::InterfaceDescription>
DynaArmHardwareInterface::export_unlisted_command_interface_descriptions()
{
}

hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& system_info)
{
  const auto arm_name = system_info.hardware_info.name;
  // The logger is now a child logger with a more descriptive name
  logger_ = logger_.get_child(arm_name);

  const auto ethercat_bus = system_info.hardware_info.hardware_parameters.at("ethercat_bus");
  // We obtain information about configured joints and create DuaDriveInterface instances from them
  // and initialize them
  for (const auto& joint : system_info.hardware_info.joints) {
    const auto address = std::stoi(joint.parameters.at("address"));
    const auto joint_name = joint.name;
    const std::string device_file_path = joint.parameters.at("drive_parameter_file");
    const std::string default_parameter_file_path = joint.parameters.at("drive_parameter_folder_default") + "/example_"
                                                                                                            "drive_"
                                                                                                            "config."
                                                                                                            "yaml";

    RCLCPP_INFO_STREAM(logger_, "Setup drive instance for joint: " << joint_name << " on ethercat bus:" << ethercat_bus
                                                                   << " at address: " << address);
    drives_.emplace_back(DuaDriveInterface{ logger_ });
    // As we need to apply the kinematic translation we need some place to store the corresponding data
    // TODO(firesurfer) we could ellide copies if we would directly use pointers on the state interface
    state_coupled_kinematics_.emplace_back(CoupledJointState{});
    state_serial_kinematics_.emplace_back(SerialJointState{});

    commands_coupled_kinematics_.emplace_back(SerialCommand{});
    commands_serial_kinematics_.emplace_back(CoupledCommand{});
    // Init doesn't really do anything apart from setting parameters
    drives_.back().init(DuaDriveInterfaceParameters{ .ethercat_bus = ethercat_bus,
                                                     .joint_name = joint_name,
                                                     .drive_parameter_file_path = device_file_path,
                                                     .drive_default_parameter_file_path = default_parameter_file_path,
                                                     .device_address = address });
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  for (auto& drive : drives_) {
    // Call configure for each drive and propagate errors if necessary
    // We currently treat every error that can happen in this stage as fatal
    if (drive.configure() != hardware_interface::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL_STREAM(logger_, "Failed to 'configure' drive: " << drive.get_name() << ". Aborting startup!");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  for (auto& drive : drives_) {
    // Call activate for each drive and propagate errors if necessary
    // We currently treat every error that can happen in this stage as fatal
    if (drive.activate() != hardware_interface::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL_STREAM(logger_, "Failed to 'activate' drive: " << drive.get_name() << ". Aborting startup!");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynaArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (auto& drive : drives_) {
    // Call deactivate for each drive and propagate errors if necessary
    if (drive.deactivate() != hardware_interface::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL_STREAM(logger_, "Failed to 'deactivate' drive: " << drive.get_name() << ". Aborting startup!");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynaArmHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // TODO(firesurfer) - replace with std::views::zip (or own implementation) when available
  for (std::size_t i = 0; i < drives_.size(); i++) {
    auto& drive = drives_[i];
    auto& state = state_coupled_kinematics_[i];
    // Try to read from each drive - in case of an error
    if (drive.read() != hardware_interface::return_type::OK) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to 'read' from drive: " << drive.get_name());
      return hardware_interface::return_type::ERROR;
    }

    // Update the coupled state (one could call this motor readings)
    const auto& latest_reading = drive.get_last_state();
    state.position = latest_reading.joint_position;
    state.velocity = latest_reading.joint_velocity;
    state.acceleration = latest_reading.joint_acceleration;
    state.torque = latest_reading.joint_torque;
  }

  // Now comes the interesting part. We need to take the data from each drive and apply the serial linkage
  // NOTE: This assumes the joints are declared in the correct order (No clue how I could validate that)
  kinematics::map_from_coupled_to_serial(state_coupled_kinematics_, state_serial_kinematics_);
  // Perform the update of the exposed state interface
  update_state_interfaces(state_interface_mapping_, *this);

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type DynaArmHardwareInterface::write(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
{
  // Get commands from the ros2control exposed command interfaces
  update_command_interfaces(command_interface_mapping_, *this);
  // Translated commands to coupled kinematics
  kinematics::map_from_serial_to_coupled(commands_serial_kinematics_, commands_coupled_kinematics_);

  for (std::size_t i = 0; i < drives_.size(); i++) {
    auto& drive = drives_[i];
    auto& cmd = commands_coupled_kinematics_[i];

    auto command = drive.get_last_command();
    command.joint_position = cmd.position;
    command.joint_velocity = cmd.velocity;
    command.joint_acceleration = cmd.acceleration;
    command.joint_torque = cmd.torque;

    drive.stage_command(command);
  }
}

hardware_interface::return_type DynaArmHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
}

hardware_interface::return_type DynaArmHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
}

DynaArmHardwareInterface::~DynaArmHardwareInterface()
{
  RCLCPP_INFO_STREAM(logger_, "Destructor of DynaArm Hardware Interface called");
}
}  // namespace duatic_dynaarm_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(duatic_dynaarm_driver::DynaArmHardwareInterface, hardware_interface::SystemInterface)
