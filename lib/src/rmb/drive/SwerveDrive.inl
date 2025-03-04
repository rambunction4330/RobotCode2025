#pragma once

#include "frc/TimedRobot.h"
#include "frc/Watchdog.h"
#include "frc/geometry/Pose2d.h"
#include "pathplanner/lib/path/PathConstraints.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
//#include "pathplanner/lib/util/HolonomicPathFollowerConfig.h"
//#include "pathplanner/lib/util/ReplanningConfig.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"

#include "frc/controller/HolonomicDriveController.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"

#include "frc2/command//SwerveControllerCommand.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/Subsystem.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include "rmb/drive/SwerveDrive.h"
#include "rmb/drive/SwerveModule.h"

#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"

#include "wpi/array.h"

#include <array>
#include <cstddef>
#include <initializer_list>

#include <Eigen/Core>

#include <iostream>
#include <memory>
#include <mutex>

//#include "pathplanner/lib/commands/FollowPathHolonomic.h"
//#include "pathplanner/lib/commands/PathfindHolonomic.h"
#include "pathplanner/lib/path/PathConstraints.h"
//#include "pathplanner/lib/util/HolonomicPathFollowerConfig.h"
//#include "pathplanner/lib/util/ReplanningConfig.h"

namespace rmb {

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<rmb::Gyro> gyro,
    frc::HolonomicDriveController holonomicController, std::string visionTable,
    units::meters_per_second_t maxModuleSpeed, const frc::Pose2d &initialPose)
    : modules(std::move(modules)), gyro(gyro),
      kinematics(getModuleTranslations()),
      holonomicController(holonomicController),
      poseEstimator(frc::SwerveDrivePoseEstimator<NumModules>(
          kinematics, gyro->getRotation(), getModulePositions(), initialPose)),
      maxModuleSpeed(maxModuleSpeed),
      watchdog(frc::TimedRobot::kDefaultPeriod,
               []() { std::cout << "SwerveDrive<NumModules> loop overrun!"; }) {
  nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("swervedrive");

  ntPositionTopics = table->GetDoubleArrayTopic("mod_positions").Publish();
  ntVelocityTopics = table->GetDoubleArrayTopic("mod_velocities").Publish();

  ntPositionErrorTopics =
      table->GetDoubleArrayTopic("mod_position_errors").Publish();
  ntVelocityErrorTopics =
      table->GetDoubleArrayTopic("mod_velocity_errors").Publish();

  ntTargetPositionTopics =
      table->GetDoubleArrayTopic("mod_position_targets").Publish();
  ntTargetVelocityTopics =
      table->GetDoubleArrayTopic("mod_velocity_targets").Publish();

  ntVelocityVoltages =
      table->GetDoubleArrayTopic("mod_velocity_voltages").Publish();
  ntVelocityCurrents =
      table->GetDoubleArrayTopic("mod_velocity_currents").Publish();

  units::meter_t maxDistance = 0.0_m;
  for (SwerveModule &module : this->modules) {
    auto &translation = module.getModuleTranslation();
    units::meter_t distance =
        translation.Distance(frc::Translation2d(0.0_m, 0.0_m));

    maxDistance = units::math::max(distance, maxDistance);
  }

  largestModuleDistance = maxDistance;
}

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<rmb::Gyro> gyro,
    frc::HolonomicDriveController holonomicController,
    units::meters_per_second_t maxModuleSpeed, const frc::Pose2d &initialPose)
    : SwerveDrive(std::move(modules), gyro, holonomicController, "",
                  maxModuleSpeed, initialPose) {}

template <size_t NumModules>
std::array<frc::SwerveModulePosition, NumModules>
SwerveDrive<NumModules>::getModulePositions() const {
  std::array<frc::SwerveModulePosition, NumModules> states;
  for (size_t i = 0; i < NumModules; i++) {
    states[i] = modules[i].getPosition();
  }

  return states;
}

template <size_t NumModules>
std::array<frc::Translation2d, NumModules>
SwerveDrive<NumModules>::getModuleTranslations() const {
  std::array<frc::Translation2d, NumModules> states;
  for (size_t i = 0; i < NumModules; i++) {
    states[i] = modules[i].getTranslation();
  }

  return states;
}

template <size_t NumModules>
std::array<frc::SwerveModuleState, NumModules>
SwerveDrive<NumModules>::getModuleStates() const {
  std::array<frc::SwerveModuleState, NumModules> states;
  for (size_t i = 0; i < NumModules; i++) {
    states[i] = modules[i].getState();
  }

  return states;
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveCartesian(double xSpeed, double ySpeed,
                                             double zRotation,
                                             bool fieldOriented) {

  Eigen::Vector2d robotRelativeVXY = Eigen::Vector2d(xSpeed, ySpeed);

  if (fieldOriented) {
    units::radian_t vXYRotationAngle = -gyro->getRotation().Radians();

    Eigen::Matrix2d vXYRotation{
        {std::cos(vXYRotationAngle()), -std::sin(vXYRotationAngle())},
        {std::sin(vXYRotationAngle()), std::cos(vXYRotationAngle())}};

    robotRelativeVXY = vXYRotation * robotRelativeVXY;
  }

  /**
   * https://dominik.win/blog/programming-swerve-drive/
   *
   * output = \vec{v} + w * perpendicular{m}
   * where v is the net translational velocity and m is the module's translation
   * from the center
   *
   * And we define our perpendicular functions as
   * perpendicular(x, y) => (-y, x)
   * to enforce a counterclockwise positive angle
   *
   * so,
   * output_x = vx * 1 + vy * 0 + w * -y
   * output_y = vx * 0 + vy * 1 + w * x
   */
  std::cout << "inputs: (" << xSpeed << ", " << ySpeed << ", " << zRotation
            << ")" << std::endl;

  std::array<SwerveModulePower, NumModules> powers;
  double largestPower = 1.0;

  // std::cout << "rotation: [";
  for (size_t i = 0; i < modules.size(); i++) {
    SwerveModule &module = modules[i];

    double output_x =
        robotRelativeVXY.x() +
        zRotation * (module.getModuleTranslation().Y() / largestModuleDistance);
    // -1 * 1

    double output_y = robotRelativeVXY.y() +
                      zRotation * -1 * module.getModuleTranslation().X() /
                          largestModuleDistance;
    //
    // std::cout << "output: (" << output_x << ", " << output_y << ")"
    //           << std::endl;

    // -1 * -1

    frc::Rotation2d moduleRotation =
        units::radian_t(std::atan2(output_y, output_x));

    

    // std::cout << "(" << robotRelativeVXY.x() << ", " << robotRelativeVXY.y()
    //           << ")" << moduleRotation.Degrees()() << ", ";
    //
    // std::cout << "moduleRotation[" << i << "] = " <<
    // moduleRotation.Degrees()()
    //           << std::endl;

    // std::cout << "output[" << i << "] = " << output_x << " " << output_y <<
    // std::endl;
    double modulePower = std::sqrt(output_x * output_x + output_y * output_y);

    powers[i] = SwerveModulePower{modulePower, moduleRotation};

    if (std::abs(modulePower) > std::abs(largestPower)) {
      largestPower = std::abs(modulePower);
    }
  }

  // std::cout << "]" << std::endl;

  // Desaturize
  for (SwerveModulePower &power : powers) {
    power.power /= std::abs(largestPower);
  }

  // Optimize
  for (size_t i = 0; i < modules.size(); i++) {
    powers[i] =
        SwerveModulePower::Optimize(powers[i], modules[i].getState().angle);
  }

  driveModulePowers(powers);

  watchdog.AddEpoch("SwerveDrive<" + std::to_string(NumModules) +
                    ">.driveCartesian");
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveCartesian(
    units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
    units::turns_per_second_t zRotation, bool fieldOriented) {
  driveChassisSpeeds(frc::ChassisSpeeds{xSpeed, ySpeed, zRotation},
                     fieldOriented);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModuleStates(
    std::array<frc::SwerveModuleState, NumModules> states) {
  for (size_t i = 0; i < NumModules; i++) {
    modules[i].setState(states[i]);
  }
}

template <size_t NumModules>
void SwerveDrive<NumModules>::drivePolar(double speed,
                                         const frc::Rotation2d &angle,
                                         double zRotation, bool fieldOriented) {
  double vx = speed * angle.Cos();
  double vy = speed * angle.Sin();

  driveCartesian(vx, vy, zRotation, fieldOriented);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::drivePolar(units::meters_per_second_t speed,
                                         const frc::Rotation2d &angle,
                                         units::turns_per_second_t omega,
                                         bool fieldOriented) {
  units::meters_per_second_t vx = speed * angle.Cos();
  units::meters_per_second_t vy = speed * angle.Sin();

  driveCartesian(vx, vy, omega, fieldOriented);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModulePowers(
    std::array<SwerveModulePower, NumModules> powers) {
  // std::cout << "powers: ";
  for (size_t i = 0; i < NumModules; i++) {
    modules[i].setPower(powers[i]);
  }
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveChassisSpeeds(
    frc::ChassisSpeeds chassisSpeeds) {
  auto states = kinematics.ToSwerveModuleStates(chassisSpeeds);
  // std::cout << "rotation: "
  //           << ((units::turns_per_second_t)chassisSpeeds.omega)() <<
  //           std::endl;
  kinematics.DesaturateWheelSpeeds(&states, maxModuleSpeed);
  driveModuleStates(states);

  watchdog.AddEpoch("SwerveDrive<" + std::to_string(NumModules) +
                    ">.driveChassisSpeeds");
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveChassisSpeeds(
    frc::ChassisSpeeds chassisSpeeds, bool fieldRelative) {

  if (fieldRelative)
    chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        chassisSpeeds, gyro->getRotation());

  driveChassisSpeeds(chassisSpeeds);
}

// template<size_t NumModules>
// void SwerveDrive<NumModules>::turnModuleAsCenter(frc::ChassisSpeeds
// chassisSpeeds, frc::Translation2d centerOfRotation){
//  auto states = kinematics.ToSwerveModuleStates(chassisSpeeds,
//  centerOfRotation);

// }

template <size_t NumModules>
frc::ChassisSpeeds SwerveDrive<NumModules>::getChassisSpeeds() const {
  auto speeds = kinematics.ToChassisSpeeds(
      wpi::array<frc::SwerveModuleState, NumModules>(getModuleStates()));

  // speeds.omega *= -1;

  return speeds;
}
template <size_t NumModules>
void SwerveDrive<NumModules>::turnAroundModule(size_t module,
                                               units::turns_per_second_t v) {
  auto speeds = kinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{.vx = 0.0_mps, .vy = 0.0_mps, .omega = v},
      modules[module].getTranslation());

  driveModuleStates(speeds);
}

template <size_t NumModules>
frc::Pose2d SwerveDrive<NumModules>::getPose() const {
  // std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.GetEstimatedPosition();
}

template <size_t NumModules> frc::Pose2d SwerveDrive<NumModules>::updatePose() {
  // std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.Update(gyro->getRotation(), getModulePositions());
}

template <size_t NumModules>
void SwerveDrive<NumModules>::updateNTDebugInfo(bool openLoopVelocity) {
  std::array<double, NumModules> velocityErrors;
  for (size_t i = 0; i < NumModules; i++) {
    units::meters_per_second_t error = 0.0_mps;
    if (!openLoopVelocity) {
      error = modules[i].getTargetState().speed - modules[i].getState().speed;
    }

    velocityErrors[i] = std::abs(error());
  }
  ntVelocityErrorTopics.Set(velocityErrors);

  std::array<double, NumModules> positionErrors;
  for (size_t i = 0; i < NumModules; i++) {
    units::turn_t rotation = units::math::fmod(
        units::math::abs(modules[i].getState().angle.Degrees()), 180.0_deg);
    // rotation = wpi::sgn(rotation()) *
    //            units::math::min(units::math::abs(rotation),
    //                             180_deg - units::math::abs(rotation));

    units::turn_t target = units::math::fmod(
        units::math::abs(modules[i].getTargetRotation().Degrees()), 180.0_deg);
    // target = wpi::sgn(target()) *
    //            units::math::min(units::math::abs(target),
    //                             180_deg - units::math::abs(target));

    units::degree_t error =
        units::math::min(units::math::abs(target - rotation),
                         units::math::abs(target - rotation));
    // modules[i].getState().angle.Degrees();

    positionErrors[i] = error();
  }
  ntPositionErrorTopics.Set(positionErrors);

  std::array<double, NumModules> velocityTargets;
  for (size_t i = 0; i < NumModules; i++) {
    double target = 0.0;

    if (!openLoopVelocity) {
      target = modules[i].getTargetVelocity()();
    } else {
      target = modules[i].getPower().power;
    }

    velocityTargets[i] = target;
  }
  ntTargetVelocityTopics.Set(velocityTargets);

  std::array<double, NumModules> positionTargets;
  for (size_t i = 0; i < NumModules; i++) {
    units::degree_t target = modules[i].getTargetRotation().Degrees();

    positionTargets[i] = target();
  }
  ntTargetPositionTopics.Set(positionTargets);

  std::array<double, NumModules> positions;
  for (size_t i = 0; i < NumModules; i++) {
    units::degree_t position = modules[i].getState().angle.Degrees();

    positions[i] = position();
  }
  ntPositionTopics.Set(positions);

  std::array<double, NumModules> velocities;
  for (size_t i = 0; i < NumModules; i++) {
    units::meters_per_second_t velocity = modules[i].getState().speed;

    velocities[i] = std::abs(velocity());
  }
  ntVelocityTopics.Set(velocities);

  std::array<double, NumModules> velocityVoltages;
  for (size_t i = 0; i < NumModules; i++) {
    velocityVoltages[i] = std::abs(modules[i].getPower().power);
  }
  ntVelocityVoltages.Set(velocityVoltages);
}

template <size_t NumModules>
std::array<frc::SwerveModuleState, NumModules>
SwerveDrive<NumModules>::getTargetModuleStates() const {
  std::array<frc::SwerveModuleState, NumModules> targetStates;
  for (size_t i = 0; i < NumModules; i++) {
    targetStates[i] = modules[i].getTargetState();
  }

  return targetStates;
}

template <size_t NumModules>
void SwerveDrive<NumModules>::resetPose(const frc::Pose2d &pose) {
  // std::lock_guard<std::mutex> lock(visionThreadMutex);
  gyro->resetZRotation();
  gyro->setZRotationOffset(pose.Rotation());

  poseEstimator.ResetPosition(gyro->getRotation(), getModulePositions(), pose);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::addVisionMeasurments(
    const frc::Pose2d &poseEstimate, units::second_t time) {
  // std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.AddVisionMeasurement(poseEstimate, time);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::setVisionSTDevs(
    wpi::array<double, 3> standardDevs) {
  // std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.SetVisionMeasurementStdDevs(standardDevs);
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::followWPILibTrajectory(
    frc::Trajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  return frc2::SwerveControllerCommand<NumModules>(
             trajectory, [this]() { return getPose(); }, kinematics,
             holonomicController,
             [this](std::array<frc::SwerveModuleState, NumModules> states) {
               driveModuleStates(states);
             },
             driveRequirements)
      .ToPtr();
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::followPPPath(
    std::shared_ptr<pathplanner::PathPlannerPath> path,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  // pathplanner::ReplanningConfig replanningConfig;
  // units::second_t period;
  // pathplanner::HolonomicPathFollowerConfig holonomicPathFollowerConfig(
  //     maxModuleSpeed, largestModuleDistance, replanningConfig, period);

  // return pathplanner::FollowPathHolonomic(
  //            path, [this]() { return getPose(); },
  //            [this]() { return getChassisSpeeds(); },
  //            [this](frc::ChassisSpeeds chassisSpeeds) {
  //              driveChassisSpeeds(chassisSpeeds);
  //            },
  //            holonomicPathFollowerConfig, []() { return true; },
  //            driveRequirements)
  //     .ToPtr();
  return frc2::cmd::None(); 
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::FollowGeneratedPPPath(
    frc::Pose2d targetPose, pathplanner::PathConstraints contraints,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  // pathplanner::ReplanningConfig replanningConfig;
  // units::second_t period;
  // pathplanner::HolonomicPathFollowerConfig holonomicPathFollowerConfig(
  //     maxModuleSpeed, largestModuleDistance, replanningConfig, period);
  // units::meter_t rotationDelayDistance = 0_m;

  //return pathplanner::PathfindHolonomic(
      //        targetPose, contraints, [this]() { return getPose(); },
      //        [this]() { return getChassisSpeeds(); },
      //        [this](frc::ChassisSpeeds chassisSpeeds) {
      //          driveChassisSpeeds(chassisSpeeds);
      //        },
      //        holonomicPathFollowerConfig, driveRequirements,
      //        rotationDelayDistance)
      // .ToPtr();
      return frc2::cmd::None();  
}

template <size_t NumModules> void SwerveDrive<NumModules>::stop() {
  std::cout << "stop right now!!!" << std::endl;
  for (auto &module : modules) {
    module.stop();
  }
}

} // namespace rmb
