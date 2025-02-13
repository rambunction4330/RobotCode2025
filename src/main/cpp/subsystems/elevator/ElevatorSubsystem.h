// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "ElevatorConstants.h"
#include <frc2/command/SubsystemBase.h>
#include <memory>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

rmb::TalonFXPositionController elevator1; 
rmb::TalonFXPositionController elevator2; 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
