// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "frc/XboxController.h"
#include "frc/event/BooleanEvent.h"
#include "frc/event/EventLoop.h"
#include "frc2/command/button/Trigger.h"
#include "main/cpp/subsystems/drive/DriveSubsystem.h"
#include "main/cpp/subsystems/elevator/ElevatorSubsystem.h"
#include "rmb/controller/LogitechGamepad.h"
#include "rmb/sensors/AHRS/AHRSGyro.h"
#include "subsystems/ExampleSubsystem.h"
#include <functional>
#include <memory>
#include <rev/SparkMax.h>
//#include <frc/Joystick.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  frc2::CommandPtr runMotorCommand(); 

  void setTeleopDefaults(); 


 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  std::shared_ptr<rmb::NavXGyro::Gyro> gyro = std::make_shared<rmb::NavXGyro>(OperatorConstants::gyroPort);
  
  DriveSubsystem driveSubsystem; 
  ElevatorSubsystem elevatorSubsystem; 


  // rmb::LogitechGamepad gamepad{
  //     OperatorConstants::driverControllerPort};
  frc::XboxController controller {0};
  // rmb::LogitechGamepad elevatorGamepad{1}; 

  frc::EventLoop *loop; 
  frc2::Trigger m_trigger; 

  void ConfigureBindings();

};
