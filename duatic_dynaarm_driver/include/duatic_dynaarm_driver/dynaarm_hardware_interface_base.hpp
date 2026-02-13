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

#pragma once

// System
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <filesystem>

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ROS
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "duatic_dynaarm_driver/kinematic_translation.hpp"

// sdk
#include <duatic_ros2control_hardware/interface_utils.hpp>
#include <duatic_ros2control_hardware/duadrive_interface_base.hpp>
#include <duatic_dynaarm_driver/types.hpp>

namespace duatic_dynaarm_driver
{
template <typename DriveTypeT>
class DynaArmHardwareInterfaceBase : public hardware_interface::SystemInterface
{
public:
  DynaArmHardwareInterfaceBase() : logger_(rclcpp::get_logger("DynaArmHardwareInterfaceBase"))
  {
  }

  std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions() override
  {
    // This is rather simple - we just append all state interface descriptions of each drive and are done
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;

    for (const auto& drive : drives_) {
      RCLCPP_INFO_STREAM(logger_, "Setting up state interfaces for drive: " << drive->get_name());
      const auto drive_state_interfaces = drive->get_state_interface_descriptions();
      state_interfaces.insert(state_interfaces.end(), drive_state_interfaces.begin(), drive_state_interfaces.end());
    }

    // More interesting is now the internal state data mapping
    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_[i];
      auto& state = state_serial_kinematics_[i];

      auto state_mapping = drive->get_default_state_mapping();
      // Now comes the magic - we replace the "position, velocity, acceleration, torque" fields (+ their commanded
      // counterparts) with our local ones that have the translated kinematics
      state_mapping[duatic_ros2control_hardware::get_interface_name(
          drive->get_name(), hardware_interface::HW_IF_POSITION)] = &state.position;
      state_mapping[duatic_ros2control_hardware::get_interface_name(
          drive->get_name(), hardware_interface::HW_IF_VELOCITY)] = &state.velocity;
      state_mapping[duatic_ros2control_hardware::get_interface_name(
          drive->get_name(), hardware_interface::HW_IF_ACCELERATION)] = &state.acceleration;
      state_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(),
                                                                    hardware_interface::HW_IF_EFFORT)] = &state.torque;

      state_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(), "position_commanded")] =
          &state.position_commanded;
      state_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(), "velocity_commanded")] =
          &state.velocity_commanded;
      state_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(), "acceleration_commanded")] =
          &state.acceleration_commanded;
      state_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(), "effort_commanded")] =
          &state.torque_commanded;

      // Append to our interface global mapping (which is used later on in the configure step to create our final
      // mapping)
      state_interface_pre_mapping_.insert(state_mapping.begin(), state_mapping.end());
    }

    return state_interfaces;
  }
  std::vector<hardware_interface::InterfaceDescription> export_unlisted_command_interface_descriptions() override
  {
    std::vector<hardware_interface::InterfaceDescription> command_interfaces;

    for (const auto& drive : drives_) {
      RCLCPP_INFO_STREAM(logger_, "Setting up command interfaces for drive: " << drive->get_name());
      const auto drive_command_interfaces = drive->get_command_interface_descriptions();
      RCLCPP_INFO_STREAM(logger_, "size: " << drive_command_interfaces.size());
      command_interfaces.insert(command_interfaces.end(), drive_command_interfaces.begin(),
                                drive_command_interfaces.end());
    }
    // Append the arm wide freeze mode field
    command_interfaces.emplace_back(duatic_ros2control_hardware::create_interface_description<bool>(
        this->get_hardware_info().name, "freeze_mode", true));

    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_[i];
      auto& cmd = commands_serial_kinematics_[i];

      auto cmd_mapping = drive->get_default_command_mapping();
      // Now comes the magic - we replace the "position, velocity, acceleration, torque" fields (+ their commanded
      // counterparts) with our local ones that have the translated kinematics
      cmd_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(),
                                                                  hardware_interface::HW_IF_POSITION)] = &cmd.position;
      cmd_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(),
                                                                  hardware_interface::HW_IF_VELOCITY)] = &cmd.velocity;
      cmd_mapping[duatic_ros2control_hardware::get_interface_name(
          drive->get_name(), hardware_interface::HW_IF_ACCELERATION)] = &cmd.acceleration;
      cmd_mapping[duatic_ros2control_hardware::get_interface_name(drive->get_name(),
                                                                  hardware_interface::HW_IF_EFFORT)] = &cmd.torque;

      // Append to our interface global mapping (which is used later on in the configure step to create our final
      // mapping)
      command_interface_pre_mapping_.insert(cmd_mapping.begin(), cmd_mapping.end());
    }

    for (auto& cmd_interface : command_interfaces) {
      RCLCPP_INFO_STREAM(logger_, cmd_interface.get_name());
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams& system_info) override
  {
    RCLCPP_INFO_STREAM(logger_, "on_init");
    const auto arm_name = system_info.hardware_info.name;
    // The logger is now a child logger with a more descriptive name
    logger_ = logger_.get_child(arm_name);

    const auto ethercat_bus = system_info.hardware_info.hardware_parameters.at("ethercat_bus");

    RCLCPP_INFO_STREAM(logger_, "Start with drives");
    // We obtain information about configured joints and create DuaDriveInterface instances from them
    // and initialize them
    for (const auto& joint : system_info.hardware_info.joints) {
      const auto address = std::stoi(joint.parameters.at("address"));
      const auto joint_name = joint.name;

      // TODO(firesurfer) replace with per drive config file path
      // We do not want the joint prefix in the path for the parameter files
      auto joint_wo_prefix = joint_name;
      const auto tf_prefix = system_info.hardware_info.hardware_parameters.at("tf_prefix");
      // Check if the joint name starts with the tf_prefix
      if (joint_name.find(tf_prefix) == 0) {
        // Remove it from the string
        joint_wo_prefix.erase(0, tf_prefix.size());
      }

      // Obtain the parameter file for the currently processed drive
      const std::string base_directory = info_.hardware_parameters.at("drive_parameter_folder") + "/";
      std::string device_file_path = base_directory + joint_wo_prefix + ".yaml";
      // If there is no configuration available for the current joint in the passed parameter folder we load it from the
      // default folder
      if (!std::filesystem::exists(device_file_path)) {
        RCLCPP_WARN_STREAM(logger_, "No configuration found for joint: " << joint_name << " in: " << base_directory
                                                                         << " Loading drive parameters from default "
                                                                            "location");

        device_file_path =
            info_.hardware_parameters.at("drive_parameter_folder_default") + "/" + joint_wo_prefix + ".yaml";
      }

      RCLCPP_INFO_STREAM(logger_, "Setup drive instance for joint: " << joint_name << " on ethercat bus:"
                                                                     << ethercat_bus << " at address: " << address);
      drives_.emplace_back(std::make_unique<DriveTypeT>(logger_));
      // As we need to apply the kinematic translation we need some place to store the corresponding data
      // TODO(firesurfer) we could ellide copies if we would directly use pointers on the state interface
      state_coupled_kinematics_.emplace_back(CoupledJointState{});
      state_serial_kinematics_.emplace_back(SerialJointState{});

      commands_coupled_kinematics_.emplace_back(SerialCommand{});
      commands_serial_kinematics_.emplace_back(CoupledCommand{});
      // Init doesn't really do anything apart from setting parameters
      drives_.back()->init(duatic_ros2control_hardware::DuaDriveInterfaceParameters{
          .ethercat_bus = ethercat_bus,
          .joint_name = joint_name,
          .drive_parameter_file_path = device_file_path,
          .drive_default_parameter_file_path = device_file_path,
          .device_address = address });
    }

    RCLCPP_INFO_STREAM(logger_, "Done setting up drives");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) override
  {
    for (auto& drive : drives_) {
      // Call configure for each drive and propagate errors if necessary
      // We currently treat every error that can happen in this stage as fatal
      if (drive->configure() != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL_STREAM(logger_, "Failed to 'configure' drive: " << drive->get_name() << ". Aborting startup!");
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }

    // And obtain the handle for the freeze mode interface
    freeze_mode_interface_ = get_command_interface_handle(get_hardware_info().name + "/freeze_mode");

    state_interface_mapping_ = create_state_interface_mapping(state_interface_pre_mapping_, *this);
    command_interface_mapping_ = create_command_interface_mapping(command_interface_pre_mapping_, *this);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) override
  {
    for (auto& drive : drives_) {
      // Call activate for each drive and propagate errors if necessary
      // We currently treat every error that can happen in this stage as fatal
      if (drive->activate() != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL_STREAM(logger_, "Failed to 'activate' drive: " << drive->get_name() << ". Aborting startup!");
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    for (auto& drive : drives_) {
      // Call deactivate for each drive and propagate errors if necessary
      if (drive->deactivate() != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL_STREAM(logger_, "Failed to 'deactivate' drive: " << drive->get_name() << ". Aborting startup!");
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read([[maybe_unused]] const rclcpp::Time& time,
                                       [[maybe_unused]] const rclcpp::Duration& period) override
  {
    // TODO(firesurfer) - replace with std::views::zip (or own implementation) when available
    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_[i];
      auto& state = state_coupled_kinematics_[i];
      // Try to read from each drive - in case of an error
      if (drive->read() != hardware_interface::return_type::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to 'read' from drive: " << drive->get_name());
        return hardware_interface::return_type::ERROR;
      }

      // Update the coupled state (one could call this motor readings)
      const auto& latest_reading = drive->get_last_state();
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
  hardware_interface::return_type write([[maybe_unused]] const rclcpp::Time& time,
                                        [[maybe_unused]] const rclcpp::Duration& period) override
  {
    // Get commands from the ros2control exposed command interfaces
    update_command_interfaces(command_interface_mapping_, *this);
    // Translated commands to coupled kinematics
    kinematics::map_from_serial_to_coupled(commands_serial_kinematics_, commands_coupled_kinematics_);

    // TODO(firesurfer) port fancy self collision avoidance logic

    const bool enforced_freeze = freeze_mode_interface_->get_optional<bool>().value();
    // Stage all commands
    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_[i];
      auto& cmd = commands_coupled_kinematics_[i];

      auto command = drive->get_last_command();
      command.joint_position = cmd.position;
      command.joint_velocity = cmd.velocity;
      command.joint_acceleration = cmd.acceleration;
      command.joint_torque = cmd.torque;
      // Forward the enforced freeze mode here
      command.joint_freeze_mode = enforced_freeze;

      drive->stage_command(command);
    }

    // Perform actual write action
    // Note this is still asynchronous to the bus communication
    for (auto& drive : drives_) {
      if (drive->write() != hardware_interface::return_type::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to 'write' to drive: " << drive->get_name());
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override
  {
    const auto next_mode = duatic_ros2control_hardware::select_mode(start_interfaces, stop_interfaces, logger_);
    RCLCPP_INFO_STREAM(logger_, "Selecting new control mode: " << next_mode);
    for (auto& drive : drives_) {
      drive->configure_drive_mode(next_mode);
    }
    return hardware_interface::return_type::OK;
  }

  virtual ~DynaArmHardwareInterfaceBase()
  {
    RCLCPP_INFO_STREAM(logger_, "Destructor of DynaArm Hardware Interface called");
  }

private:
  std::vector<typename DriveTypeT::UniquePtr> drives_;
  std::vector<CoupledJointState> state_coupled_kinematics_;
  std::vector<SerialJointState> state_serial_kinematics_;

  std::vector<CoupledCommand> commands_coupled_kinematics_;
  std::vector<SerialCommand> commands_serial_kinematics_;

  std::unordered_map<std::string, duatic_ros2control_hardware::SupportedVariant> state_interface_pre_mapping_;
  std::unordered_map<std::string, duatic_ros2control_hardware::SupportedVariant> command_interface_pre_mapping_;

  duatic_ros2control_hardware::StateInterfaceMapping state_interface_mapping_;
  duatic_ros2control_hardware::CommandInterfaceMapping command_interface_mapping_;

  hardware_interface::CommandInterface::SharedPtr freeze_mode_interface_;

  rclcpp::Logger logger_;
};

}  // namespace duatic_dynaarm_driver
