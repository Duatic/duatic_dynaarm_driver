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

#include "duatic_dynaarm_driver/kinematic_translation.hpp"

// Keep eigen private to this file
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
namespace duatic_dynaarm_driver::kinematics
{

/*!
 * Compute the mapping from the absolute angles of the dynaarm to the relative angles used in the Ocs2 convention.
 * theta = relative angles
 * q = absolute angles
 * theta = mappingFromAbsoluteToRelativeAngles * q
 */
static Eigen::VectorXd map_from_coupled_to_serial_coordinates(const Eigen::VectorXd& input)
{
  int size = input.size();
  if (size < 4) {
    throw std::invalid_argument("Input vector must have at least 4 elements");
  }

  // Create a dynamic matrix with size 'size'
  Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

  // clang-format off
            // Modify the first 4x4 submatrix
            mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0, 0.0,
                                        0.0, -1.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Return the transformed vector
  return mapping * input;
}

static Eigen::VectorXd map_from_coupled_to_serial_torques(const Eigen::VectorXd& input)
{
  int size = input.size();
  if (size < 4) {
    throw std::invalid_argument("Input vector must have at least 4 elements");
  }

  // Create a dynamic matrix with size 'size'
  Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

  // clang-format off
            // Modify the first 4x4 submatrix
            mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 1.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Return the transformed vector
  return mapping * input;
}

/*!
 * Compute the mapping from the relative angles of the ocs2_convention to the absolute angles used for the dynaarm
 * theta = relative angles
 * q = absolute angles
 * q = mappingFromRelativeToAbsoluteAngles * theta
 */
static Eigen::VectorXd map_from_serial_to_coupled_coordinates(const Eigen::VectorXd& input)
{
  int size = input.size();
  if (size < 4) {
    throw std::invalid_argument("Input vector must have at least 4 elements");
  }

  // Create a dynamic matrix with size 'size'
  Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

  // clang-format off
            // Modify the first 4x4 submatrix
            mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 1.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Return the transformed vector
  return mapping * input;
}

static Eigen::VectorXd map_from_serial_to_coupled_torques(const Eigen::VectorXd& input)
{
  int size = input.size();
  if (size < 4) {
    throw std::invalid_argument("Input vector must have at least 4 elements");
  }

  // Create a dynamic matrix with size 'size'
  Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(size, size);

  // clang-format off
            // Modify the first 4x4 submatrix
            mapping.block<4, 4>(0, 0) << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, -1.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Return the transformed vector
  return mapping * input;
}

void map_from_coupled_to_serial(std::span<const CoupledJointState> input, std::span<SerialJointState> output)
{
  Eigen::VectorXd p_c;
  Eigen::VectorXd v_c;
  Eigen::VectorXd a_c;
  Eigen::VectorXd t_c;

  Eigen::VectorXd p_commanded_c;
  Eigen::VectorXd v_commanded_c;
  Eigen::VectorXd a_commanded_c;
  Eigen::VectorXd t_commanded_c;

  for (std::size_t i = 0; i < input.size(); i++) {
    p_c[i] = input[i].position;
    v_c[i] = input[i].velocity;
    a_c[i] = input[i].acceleration;
    t_c[i] = input[i].torque;

    p_commanded_c[i] = input[i].position_commanded;
    v_commanded_c[i] = input[i].velocity_commanded;
    a_commanded_c[i] = input[i].acceleration_commanded;
    t_commanded_c[i] = input[i].torque_commanded;
  }

  Eigen::VectorXd p_s = map_from_coupled_to_serial_coordinates(p_c);
  Eigen::VectorXd v_s = map_from_coupled_to_serial_coordinates(v_c);
  Eigen::VectorXd a_s = map_from_coupled_to_serial_coordinates(a_c);
  Eigen::VectorXd t_s = map_from_coupled_to_serial_torques(t_c);

  Eigen::VectorXd p_commanded_s = map_from_coupled_to_serial_coordinates(p_commanded_c);
  Eigen::VectorXd v_commanded_s = map_from_coupled_to_serial_coordinates(v_commanded_c);
  Eigen::VectorXd a_commanded_s = map_from_coupled_to_serial_coordinates(a_commanded_c);
  Eigen::VectorXd t_commanded_s = map_from_coupled_to_serial_torques(t_commanded_c);

  for (std::size_t i = 0; i < input.size(); i++) {
    output[i].position = p_s[i];
    output[i].velocity = v_s[i];
    output[i].acceleration = a_s[i];
    output[i].torque = t_s[i];

    output[i].position_commanded = p_commanded_s[i];
    output[i].velocity_commanded = v_commanded_s[i];
    output[i].acceleration_commanded = a_commanded_s[i];
    output[i].torque_commanded = t_commanded_s[i];
  }

  // TODO(firesurfer) add commanded values
}

void map_from_serial_to_coupled(std::span<const SerialJointState> input, std::span<CoupledJointState> output)
{
  Eigen::VectorXd p_s;
  Eigen::VectorXd v_s;
  Eigen::VectorXd a_s;
  Eigen::VectorXd t_s;

  Eigen::VectorXd p_commanded_s;
  Eigen::VectorXd v_commanded_s;
  Eigen::VectorXd a_commanded_s;
  Eigen::VectorXd t_commanded_s;

  for (std::size_t i = 0; i < input.size(); i++) {
    p_s[i] = input[i].position;
    v_s[i] = input[i].velocity;
    a_s[i] = input[i].acceleration;
    t_s[i] = input[i].torque;

    p_commanded_s[i] = input[i].position_commanded;
    v_commanded_s[i] = input[i].position_commanded;
    a_commanded_s[i] = input[i].position_commanded;
    t_commanded_s[i] = input[i].position_commanded;
  }

  Eigen::VectorXd p_c = map_from_serial_to_coupled_coordinates(p_s);
  Eigen::VectorXd v_c = map_from_serial_to_coupled_coordinates(p_s);
  Eigen::VectorXd a_c = map_from_serial_to_coupled_coordinates(a_s);
  Eigen::VectorXd t_c = map_from_serial_to_coupled_torques(t_s);

  Eigen::VectorXd p_commanded_c = map_from_serial_to_coupled_coordinates(p_commanded_s);
  Eigen::VectorXd v_commanded_c = map_from_serial_to_coupled_coordinates(v_commanded_s);
  Eigen::VectorXd a_commanded_c = map_from_serial_to_coupled_coordinates(a_commanded_s);
  Eigen::VectorXd t_commanded_c = map_from_serial_to_coupled_torques(t_commanded_s);

  for (std::size_t i = 0; i < input.size(); i++) {
    output[i].position = p_c[i];
    output[i].velocity = v_c[i];
    output[i].acceleration = a_c[i];
    output[i].torque = t_c[i];

    output[i].position_commanded = p_commanded_c[i];
    output[i].velocity_commanded = v_commanded_c[i];
    output[i].acceleration_commanded = a_commanded_c[i];
    output[i].torque_commanded = t_commanded_c[i];
  }
}

void map_from_coupled_to_serial(std::span<const CoupledCommand> input, std::span<SerialCommand> output)
{
  Eigen::VectorXd p_c;
  Eigen::VectorXd v_c;
  Eigen::VectorXd a_c;
  Eigen::VectorXd t_c;

  for (std::size_t i = 0; i < input.size(); i++) {
    p_c[i] = input[i].position;
    v_c[i] = input[i].velocity;
    a_c[i] = input[i].acceleration;
    t_c[i] = input[i].torque;
  }

  Eigen::VectorXd p_s = map_from_coupled_to_serial_coordinates(p_c);
  Eigen::VectorXd v_s = map_from_coupled_to_serial_coordinates(v_c);
  Eigen::VectorXd a_s = map_from_coupled_to_serial_coordinates(a_c);
  Eigen::VectorXd t_s = map_from_coupled_to_serial_torques(t_c);

  for (std::size_t i = 0; i < input.size(); i++) {
    output[i].position = p_s[i];
    output[i].velocity = v_s[i];
    output[i].acceleration = a_s[i];
    output[i].torque = t_s[i];
  }
}

void map_from_serial_to_coupled(std::span<const SerialCommand> input, std::span<CoupledCommand> output)
{
  Eigen::VectorXd p_s;
  Eigen::VectorXd v_s;
  Eigen::VectorXd a_s;
  Eigen::VectorXd t_s;

  for (std::size_t i = 0; i < input.size(); i++) {
    p_s[i] = input[i].position;
    v_s[i] = input[i].velocity;
    a_s[i] = input[i].acceleration;
    t_s[i] = input[i].torque;
  }

  Eigen::VectorXd p_c = map_from_serial_to_coupled_coordinates(p_s);
  Eigen::VectorXd v_c = map_from_serial_to_coupled_coordinates(p_s);
  Eigen::VectorXd a_c = map_from_serial_to_coupled_coordinates(a_s);
  Eigen::VectorXd t_c = map_from_serial_to_coupled_torques(t_s);

  for (std::size_t i = 0; i < input.size(); i++) {
    output[i].position = p_c[i];
    output[i].velocity = v_c[i];
    output[i].acceleration = a_c[i];
    output[i].torque = t_c[i];
  }
}

}  // namespace duatic_dynaarm_driver::kinematics
