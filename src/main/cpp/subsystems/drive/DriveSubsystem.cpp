// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
#include "DriveSubsystem.h"
#include "DriveConstants.h"
#include "frc/controller/HolonomicDriveController.h"
#include "frc/controller/PIDController.h"
#include "frc/controller/ProfiledPIDController.h"
#include "frc/geometry/Translation2d.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/RunCommand.h"
#include "rmb/controller/LogitechGamepad.h"
#include "rmb/drive/SwerveDrive.h"
#include "rmb/drive/SwerveModule.h"
#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "rmb/motorcontrol/Talon/TalonFXVelocityController.h"
#include "rmb/sensors/gyro.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include <algorithm>
#include <array>
#include <memory>

DriveSubsystem::DriveSubsystem(std::shared_ptr<rmb::Gyro> gyro) {
    std::array<rmb::SwerveModule, 4> modules = {
        rmb::SwerveModule(
            rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
            constants::drive::velocityControllerCreateInfo1), 
            constants::drive::wheelCircumference/1_tr), 
            std::make_unique<rmb::TalonFXPositionController>(
            constants::drive::positionControllerCreateInfo1),
            frc::Translation2d(
            constants::drive::robotDimX/2.0, -constants::drive::robotDimY/2.0),
            true),

        rmb::SwerveModule(
            rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
            constants::drive::velocityControllerCreateInfo2), 
            constants::drive::wheelCircumference/1_tr), 
            std::make_unique<rmb::TalonFXPositionController>(
            constants::drive::positionControllerCreateInfo2), 
            frc::Translation2d(
            constants::drive::robotDimX/2.0, constants::drive::robotDimY/2.0),
            true),

        rmb::SwerveModule(
            rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
            constants::drive::velocityControllerCreateInfo3), 
            constants::drive::wheelCircumference/1_tr), 
            std::make_unique<rmb::TalonFXPositionController>(
            constants::drive::positionControllerCreateInfo3), 
            frc::Translation2d(
            -constants::drive::robotDimX/2.0, constants::drive::robotDimY/2.0),
            true),

        rmb::SwerveModule(
            rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
            constants::drive::velocityControllerCreateInfo4), 
            constants::drive::wheelCircumference/1_tr), 
            std::make_unique<rmb::TalonFXPositionController>(
            constants::drive::positionControllerCreateInfo4), 
            frc::Translation2d(
            -constants::drive::robotDimX/2.0, -constants::drive::robotDimY/2.0),
            true)
            };

    drive = std::make_unique<rmb::SwerveDrive<4>>(
        std::move(modules), 
        gyro, frc::HolonomicDriveController(
        frc::PIDController(0.0f, 0.0f, 0.0f), 
        frc::PIDController(0.0f, 0.0f, 0.0f), 
        frc::ProfiledPIDController<units::radian>(
        1.0f, 0.0f, 0.0f, 
        frc::TrapezoidProfile<units::radian>::Constraints(
        6.28_rad_per_s, 0.2_tr/1_s/1_s))), 
        constants::drive::maxModuleSpeed); 

    drive->resetPose();

    std::cout<<"drive subsytem up"<<std::endl; 

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table =
    ntInstance.GetTable("drivesubsystem");

    anglePublisher = table->GetDoubleTopic("angle").Publish();
    targetChassisSpeedsPublisher =
    table->GetDoubleArrayTopic("targetChassis").Publish();
    chassisSpeedsPublisher = table->GetDoubleArrayTopic("chassis").Publish();

}

void DriveSubsystem::driveTeleop(const rmb::LogitechGamepad &gamepad){
    units::meters_per_second_t maxSpeed = 1_mps; 
    units::turns_per_second_t maxRotation  = 1_tps;
    // std::cout<<"yspeed controller" << gamepad.GetLeftX()<<std::endl; 
    // std::cout<<"xspeed controller"<<gamepad.GetLeftY()<<std::endl; 
    std::cout<<"rotation controller"<<gamepad.GetRightX()<<std::endl; 
    units::meters_per_second_t xSpeed = -maxSpeed*gamepad.GetLeftY(); 
    units::meters_per_second_t ySpeed = maxSpeed*gamepad.GetLeftX(); 
    units::turns_per_second_t rotation = maxRotation*gamepad.GetRightX(); 

    if(std::abs(gamepad.GetLeftX())<0.05){
        ySpeed = 0_mps; 
    }
    if(std::abs(gamepad.GetLeftY())<0.05){
        xSpeed = 0_mps; 
    }
    if(std::abs(gamepad.GetRightX())<0.05){
        rotation = 0_tps; 
    }
    drive -> driveCartesian(xSpeed,
                            ySpeed, 
                            rotation, 
                            true); 
}

void DriveSubsystem::driveTeleop(double x, double y, double z){
    drive -> driveCartesian(x, y, x, false); 
}


frc2::CommandPtr DriveSubsystem::driveTeleopCommand(const rmb::LogitechGamepad &gamepad){
    return frc2::RunCommand([&]()
    {   drive->resetPose(); 
        driveTeleop(gamepad);}, {this}).ToPtr(); 
}

frc2::CommandPtr DriveSubsystem::driveTeleopCommand(double x, double y, double z){
    return frc2::RunCommand([&]()
    {drive->driveCartesian(x, y, z, true);}, {this}).ToPtr();
}


frc2::CommandPtr DriveSubsystem::reset(){
    return frc2::InstantCommand([this](){drive ->resetPose();}, {this}).ToPtr(); 
}


// This method will be called once per scheduler run
void DriveSubsystem::Periodic() {

    drive->updateNTDebugInfo(false);
}
