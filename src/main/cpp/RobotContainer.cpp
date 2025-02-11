// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "main/cpp/subsystems/drive/DriveSubsystem.h"

#include <frc2/command/button/Trigger.h>
#include<frc2/command/CommandPtr.h>


RobotContainer::RobotContainer():driveSubsystem(gyro) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  
}

void RobotContainer::setTeleopDefaults(){
  driveSubsystem.SetDefaultCommand(driveSubsystem.driveTeleopCommand(gamepad));
}
