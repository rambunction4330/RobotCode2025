// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem()
    : elevator1(constants::elevator::elevatorControllerCreateInfo1),
      elevator2(constants::elevator::elevatorControllerCreateInfo2) {

        elevator2.follow(elevator1, false); 
      }

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}
