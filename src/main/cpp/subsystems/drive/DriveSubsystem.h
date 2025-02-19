// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc2/command/CommandPtr.h"
#include "main/cpp/subsystems/drive/DriveConstants.h"
#include "rmb/drive/SwerveDrive.h"
#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "rmb/sensors/gyro.h"
#include "rmb/controller/LogitechGamepad.h"
#include <frc2/command/SubsystemBase.h>
#include "DriveConstants.h"
#include <memory>

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem(std::shared_ptr<rmb::Gyro> gyro);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void driveTeleop(const rmb::LogitechGamepad &gamepad); 
  void driveTeleop(double x, double y, double z); 

  frc2::CommandPtr driveTeleopCommand(const rmb::LogitechGamepad &gamepad); 
  frc2::CommandPtr driveTeleopCommand(double x , double y, double z); 

  void setModules(); 
  

  void motorStop(); 

  frc2::CommandPtr reset(); 
  
  void resetCanCoder(); 

 private:
 std::unique_ptr<rmb::SwerveDrive<4>> drive; 

 //rmb::TalonFXPositionController positionController1;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
