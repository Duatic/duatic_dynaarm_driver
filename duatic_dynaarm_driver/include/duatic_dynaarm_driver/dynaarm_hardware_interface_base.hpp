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

// sdk
#include <duatic_duadrive_interface/duadrive_interface.hpp>
#include <duatic_duadrive_interface/duadrive_interface_mock.hpp>
#include <duatic_duadrive_interface/coupled_kinematics_hardware_interface.hpp>

// local
#include "duatic_dynaarm_driver/kinematic_translation.hpp"

// Create explicit instances to reduce downstream compilation effort
namespace duatic
{
extern template class duadrive_interface::CoupledKinematicsHardwareInterfaceBase<
    duadrive_interface::DuaDriveInterface, dynaarm_driver::kinematics::DynaArmKinematicsTranslator>;

extern template class duadrive_interface::CoupledKinematicsHardwareInterfaceBase<
    duadrive_interface::DuaDriveInterfaceMock, dynaarm_driver::kinematics::DynaArmKinematicsTranslator>;
}  // namespace duatic

// Create aliases for the driver interfaces
namespace duatic::dynaarm_driver
{
template <typename DriveTypeT>
using DynaArmHardwareInterfaceBase =
    duadrive_interface::CoupledKinematicsHardwareInterfaceBase<DriveTypeT, kinematics::DynaArmKinematicsTranslator>;

using DynaArmHardwareInterfaceReal =
    duadrive_interface::CoupledKinematicsHardwareInterfaceBase<duadrive_interface::DuaDriveInterface,
                                                               kinematics::DynaArmKinematicsTranslator>;
using DynaArmHardwareInterfaceMock =
    duadrive_interface::CoupledKinematicsHardwareInterfaceBase<duadrive_interface::DuaDriveInterfaceMock,
                                                               kinematics::DynaArmKinematicsTranslator>;

}  // namespace duatic::dynaarm_driver
