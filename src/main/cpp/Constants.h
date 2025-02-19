// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include "studica/AHRS.h"
#include "units/time.h"
namespace OperatorConstants {

inline constexpr int driverControllerPort = 0;

const units::millisecond_t loop = 50_ms; 

const studica::AHRS::NavXComType gyroPort = studica::AHRS::kMXP_SPI; 

}  // namespace OperatorConstants
