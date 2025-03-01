// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/XboxController.h"
#include "frc2/command/CommandPtr.h"
#include "rmb/controller/LogitechGamepad.h"
#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "ElevatorConstants.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxPositionController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"
#include "units/angle.h"
#include "units/length.h"
#include <frc2/command/SubsystemBase.h>
#include <memory>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  units::turn_t getElevatorPosition(); 
  units::turn_t getWristPosition(); 
  void setWristPosition(units::turn_t position); 
  void setElevatorPostion(units::meter_t position);
  //void inline setWristPosition(){wrist.setEncoderPosition(0.2_tr);}

  inline void setIntakePower(double power){intake.setPower(power);}
  void runIntake(const frc::XboxController &conrtoller); 

  frc2::CommandPtr intakeCommand(const frc::XboxController &conrtoller); 

  frc2::CommandPtr setElevatorCommand(units::meter_t position, const frc::XboxController &conrtoller); 

 private:

rmb::TalonFXPositionController elevator1; 
rmb::TalonFXPositionController elevator2; 
rmb::SparkMaxPositionController wrist; 
rmb::SparkMaxVelocityController intake; 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
