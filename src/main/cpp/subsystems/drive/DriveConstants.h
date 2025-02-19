#pragma once
#include "units/angle.h"
#include "units/length.h"
#include <limits>
#include <numbers>
#include <optional>
#include <rmb/motorcontrol/Talon/TalonFXPositionController.h>
#include <rmb/motorcontrol/Talon/TalonFXVelocityController.h>
#include <units/velocity.h>

namespace constants {
namespace drive {

using rmb::TalonFXPositionControllerHelper::CANCoderConfig;

// PID Configs:
const rmb::TalonFXPositionControllerHelper::PIDConfig positionPIDConfig{
    .p = 0.1, .i=0.000, .d=0.01, .ff=0.0};
const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityPIDConfig{
    .p= 0.1, .i=0.0, .d=0.0, .ff=0.0, .kV=0.0};

// Magnet Module Offsets, Wheel Circumference, Robot Diameter, and Max Module
// Speed

const units::turn_t moduleMagnetOffset1( 0.4992673);
const units::turn_t moduleMagnetOffset2(0.44824218);
const units::turn_t moduleMagnetOffset3(0.40502929);
const units::turn_t moduleMagnetOffset4(0.479003906);

const units::meter_t wheelCircumference = 4_in * std::numbers::pi;
const units::meter_t robotDimX = 2.5_ft;
const units::meter_t robotDimY = 2.5_ft;
const units::meters_per_second_t maxModuleSpeed = 1_mps;

// module 1
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo1{
    .config{.id = 12, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfig,
    .profileConfig = {.maxVelocity = 100.0_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.82f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt};

const rmb::TalonFXPositionController::CreateInfo positionConstrollerCreateInfo1{
    .config{.id = 11, .inverted = false, .brake = true},
    .pidConfig = positionPIDConfig,
    .range{.minPosition =
               -(units::radian_t)std::numeric_limits<double>::infinity(),
           .maxPosition =
               (units::radian_t)std::numeric_limits<double>::infinity(),
           .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig = CANCoderConfig{
        .id = 10, .useIntegrated = false, .magnetOffset = moduleMagnetOffset1}};

// module 2
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo2{
    .config{.id = 21, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfig,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.82f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt};

const rmb::TalonFXPositionController::CreateInfo positionConstrollerCreateInfo2{
    .config{.id = 22, .inverted = false, .brake = true},
    .pidConfig = positionPIDConfig,
    .range{.minPosition =
               -(units::radian_t)std::numeric_limits<double>::infinity(),
           .maxPosition =
               (units::radian_t)std::numeric_limits<double>::infinity(),
           .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig = CANCoderConfig{
        .id = 20, .useIntegrated = false, .magnetOffset = moduleMagnetOffset2},
};

// module 3
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo3{
    .config{.id = 31, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfig,
    .profileConfig = {.maxVelocity = 100.0_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.82f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt,
};

const rmb::TalonFXPositionController::CreateInfo positionConstrollerCreateInfo3{
    .config{.id = 32, .inverted = false, .brake = true},
    .pidConfig = positionPIDConfig,
    .range{.minPosition =
               -(units::radian_t)std::numeric_limits<double>::infinity(),
           .maxPosition =
               (units::radian_t)std::numeric_limits<double>::infinity(),
           .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig = CANCoderConfig{
        .id = 30, .useIntegrated = false, .magnetOffset = moduleMagnetOffset3},
};

// module 4
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo4{
    .config{.id = 42, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfig,
    .profileConfig = {.maxVelocity = 100.0_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.82f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt};

const rmb::TalonFXPositionController::CreateInfo positionConstrollerCreateInfo4{
    .config{.id = 41, .inverted = false, .brake = true},
    .pidConfig = positionPIDConfig,
    .range{.minPosition =
               -(units::radian_t)std::numeric_limits<double>::infinity(),
           .maxPosition =
               (units::radian_t)std::numeric_limits<double>::infinity(),
           .continuousWrap = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 1.0f,
        },
    .openLoopConfig = {},
    .currentLimits = {},
    .canCoderConfig = CANCoderConfig{
        .id = 40, .useIntegrated = false, .magnetOffset = moduleMagnetOffset4},
};

} // namespace drive
} // namespace constants