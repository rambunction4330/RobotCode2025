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



// PID Configs:
const rmb::TalonFXPositionControllerHelper::PIDConfig positionPIDConfig{
    .p = 8.0, .i=0.0003, .d=0.0, .ff=1.0};
const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityPIDConfig{
    .p= 4.0, .i=0.0, .d=0.0, .ff=0.0, .kV=0.0};
const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityPIDConfigA{
    .p= 4.0, .i=0.0, .d=0.0, .ff=0.0, .kV=0.080};

using rmb::TalonFXPositionControllerHelper::CANCoderConfig;

// Magnet Module Offsets, Wheel Circumference, Robot Diameter, and Max Module
// Speed

const units::turn_t moduleMagnetOffset1(-0.163330078+0.5); 
const units::turn_t moduleMagnetOffset2(0.028564453); 
const units::turn_t moduleMagnetOffset3(-0.13671875);
const units::turn_t moduleMagnetOffset4(0.30737304);

const units::meter_t wheelCircumference = 4_in * std::numbers::pi;
const units::meter_t robotDimX = 1.5_ft;
const units::meter_t robotDimY = 1.5_ft;
const units::meters_per_second_t maxModuleSpeed = 1_mps;

// module 1
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo1{
    .config{.id = 10, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfig,
    .profileConfig = {.maxVelocity = 100.0_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo1{
    .config{.id = 12, .inverted = false, .brake = true},
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
        .id = 11, .useIntegrated = false, .magnetOffset = moduleMagnetOffset1}};

// module 2
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo2{
    .config{.id = 20, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfigA,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo2{
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
    .canCoderConfig= CANCoderConfig{
        .id = 21, .useIntegrated = false, .magnetOffset = moduleMagnetOffset2},
};

// module 3
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo3{
    .config{.id = 30, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfigA,
    .profileConfig = {.maxVelocity = 100.0_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt,
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo3{
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
        .id = 31, .useIntegrated = false, .magnetOffset = moduleMagnetOffset3},
};

// module 4
const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo4{
    .config{.id = 40, .inverted = false, .brake = true},
    .pidConfig = velocityPIDConfig,
    .profileConfig = {.maxVelocity = 100.0_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 10000_tr_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig{.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 0.0_s},
    .currentLimits{},
    .canCoderConfig = std::nullopt};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo4{
    .config{.id = 42, .inverted = false, .brake = true},
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
        .id = 41, .useIntegrated = false, .magnetOffset = moduleMagnetOffset4},
};

} // namespace drive
} // namespace constants