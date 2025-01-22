// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc2/command/CommandPtr.h"
#include "networktables/DoubleArrayTopic.h"
#include "networktables/DoubleTopic.h"
#include "rmb/drive/SwerveDrive.h"
#include "rmb/sensors/gyro.h"
#include <frc2/command/SubsystemBase.h>
#include "rmb/controller/LogitechGamepad.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  enum brakeMode {
    MOTOR_BRAKE = 0, 
    ANGLE_BRAKE
  }; 
  DriveSubsystem(std::shared_ptr<rmb::Gyro> gyro);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void driveTeleop(const rmb::LogitechGamepad &gamepad); 
  void driveTeleop(double x , double y, double z); 

  frc2::CommandPtr driveTeleopCommand(const rmb::LogitechGamepad &gamepad); 
  frc2::CommandPtr driveTeleopCommand(double x , double y, double z); 

  void brake(brakeMode mode); 

  void motorStop(); 

  frc2::CommandPtr reset(); 

 private:

 std::unique_ptr<rmb::SwerveDrive<4>> drive; 
 nt::DoublePublisher anglePublisher; 
 nt::DoubleArrayPublisher targetChassisSpeedsPublisher; 
 nt::DoubleArrayPublisher chassisSpeedsPublisher; 

};
