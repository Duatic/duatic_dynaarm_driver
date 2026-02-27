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
#include <span>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <duatic_duadrive_interface/coupled_kinematics_types.hpp>

namespace duatic::dynaarm_driver::kinematics
{
// Bring the types into our namespace
using SerialJointState = duadrive_interface::SerialJointState;
using CoupledJointState = duadrive_interface::CoupledJointState;
using SerialCommand = duadrive_interface::SerialCommand;
using CoupledCommand = duadrive_interface::CoupledCommand;

struct DynaArmKinematicsTranslator
{
  /**
   * @brief map the system state from the coupled state to the serial equivalent
   */
  static void map_from_coupled_to_serial(std::span<const CoupledJointState> input, std::span<SerialJointState> output);
  /**
   * @brief map the system state from the serial equivalent to the coupled
   */
  static void map_from_serial_to_coupled(std::span<const SerialJointState> input, std::span<CoupledJointState> output);

  /**
   * @brief map the system state from the coupled state to the serial equivalent
   */
  static void map_from_coupled_to_serial(std::span<const CoupledCommand> input, std::span<SerialCommand> output);
  /**
   * @brief map the system state from the serial equivalent to the coupled
   */
  static void map_from_serial_to_coupled(std::span<const SerialCommand> input, std::span<CoupledCommand> output);
};

namespace impl
{
Eigen::VectorXd map_from_coupled_to_serial_coordinates(const Eigen::VectorXd& input);
Eigen::VectorXd map_from_coupled_to_serial_torques(const Eigen::VectorXd& input);
Eigen::VectorXd map_from_serial_to_coupled_coordinates(const Eigen::VectorXd& input);
Eigen::VectorXd map_from_serial_to_coupled_torques(const Eigen::VectorXd& input);
}  // namespace impl

}  // namespace duatic::dynaarm_driver::kinematics
