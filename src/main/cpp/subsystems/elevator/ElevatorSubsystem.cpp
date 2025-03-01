// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ElevatorSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/RunCommand.h"
#include "rmb/controller/LogitechGamepad.h"
#include "units/angle.h"
#include <cmath>
#include <iostream>

ElevatorSubsystem::ElevatorSubsystem()
    : elevator1(constants::elevator::elevatorControllerCreateInfo1),
      elevator2(constants::elevator::elevatorControllerCreateInfo2),
      wrist(constants::wrist::wristPositionControllerCreateInfo),
      intake(constants::wrist::intakeVelocityController) {
  wrist.setEncoderPosition(0.0_tr);
  elevator1.setEncoderPosition(0.0_tr);
  elevator2.setEncoderPosition(0.0_tr);
  elevator2.follow(elevator1, false);
}
units::turn_t ElevatorSubsystem::getElevatorPosition() {
  // std::cout << "elevator 1 position" << elevator1.getPosition().value()
  //           << std::endl;

  // std::cout << "elevator 2 position" << elevator2.getPosition().value()
  //           << std::endl;
  return elevator1.getPosition();
  return elevator2.getPosition();
}

units::turn_t ElevatorSubsystem::getWristPosition() {
  std::cout << "wrist position" << ((units::turn_t)wrist.getPosition()).value()
            << std::endl;
  // std::cout<<"target position" << ((units::turn_t)
  // wrist.getTargetPosition()).value()<<std::endl;
  return wrist.getPosition();
}

void ElevatorSubsystem::setWristPosition(units::turn_t position) {
  wrist.setPosition(
      position, constants::wrist::wrist_kG *
                    /*(units::turn_t)*/ std::sin(
                        (double)(units::turn_t)wrist.getPosition().value()));
}

void ElevatorSubsystem::setElevatorPostion(units::meter_t position) {
  elevator1.setPosition(position /
                        constants::elevator::linearToAngularGearRatio);

  // elevator1.setPosition(0.005_tr);
}

void ElevatorSubsystem::runIntake(const frc::XboxController &controller) {
  double power;
  if (controller.GetRightTriggerAxis() == 1) {
    power = 1.0;
  } else if (controller.GetLeftTriggerAxis() == 1) {
    power = -1.0;
  } else {
    power = 0.0;
  }
  setIntakePower(power);
}

frc2::CommandPtr
ElevatorSubsystem::intakeCommand(const frc::XboxController &controller) {
  return frc2::RunCommand(
             [&]() {
               runIntake(controller);
               getElevatorPosition();
               getWristPosition();
               setWristPosition(0.12_tr);
               //  if(gamepad.GetA() == true) {
               //    setElevatorPostion(10_cm);

               //  }
               //  else if(gamepad.GetA() == 0){
               //   setElevatorPostion(1_cm);

               //  }

               if (controller.GetRightBumperButton() == true) {
                 setWristPosition(0.2_tr);
               }
               if (controller.GetBButton() == true) {
                 setElevatorPostion(8.3_cm);
                 setWristPosition(0.33_tr);
               } 
               else if(controller.GetAButton() ==true){
                setElevatorPostion(12.2_cm); 
                setWristPosition(0.31_tr); 
               }
               else if(controller.GetXButton() ==true){
                setElevatorPostion(17.2_cm); 
                // setWristPosition(0.4_tr); 
               }
               else if (controller.GetBButton() == false || controller.GetAButton()==false ||controller.GetXButton()==false) {
                 setElevatorPostion(1_cm);
                 // setWristPosition(0.06_tr);
               }

               //  if(gamepad.GetX() == true) {
               //    setElevatorPostion(17_cm);
               //  }
               //  else if(gamepad.GetX() == 0){
               //   setElevatorPostion(1_cm);
               //  }
             },
             {this})
      .ToPtr();
}

frc2::CommandPtr
ElevatorSubsystem::setElevatorCommand(units::meter_t position,
                                      const frc::XboxController &conrtoller) {
  return frc2::RunCommand([&]() { setElevatorPostion(position); }).ToPtr();
}
// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}
