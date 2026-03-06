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
#include <duatic_duadrive_interface/coupled_kinematics_translator.hpp>

namespace duatic::dynaarm_driver::kinematics
{
struct DynaArmKinematicsMapping
{
  using VectorType = Eigen::VectorXd;
  static VectorType map_from_coupled_to_serial_coordinates(const VectorType& input);
  static VectorType map_from_coupled_to_serial_torques(const VectorType& input);
  static VectorType map_from_serial_to_coupled_coordinates(const VectorType& input);
  static VectorType map_from_serial_to_coupled_torques(const VectorType& input);

  static constexpr std::size_t input_size()
  {
    return 6;
  }
};

}  // namespace duatic::dynaarm_driver::kinematics
