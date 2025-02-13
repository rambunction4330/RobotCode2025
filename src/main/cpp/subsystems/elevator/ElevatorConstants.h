

#include "rev/SparkMax.h"
#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxPositionController.h"
#include "units/angle.h"
#include "units/length.h"
namespace constants {
namespace elevator {

const rmb::TalonFXPositionControllerHelper::PIDConfig elevatorPID {.p =0.0, .i = 0.0, .d = 0.0, .ff = 0.0};
const units::turn_t maxTurns = 0.0_tr; 
const units::meter_t maxHeight = 68.5_cm;  
const auto linearToAngularGearRatio = maxHeight/maxTurns;
const rmb::TalonFXPositionController::CreateInfo elevatorControllerCreateInfo1{
    .config{.id = 61, .inverted = false, .brake = true}, 
    .pidConfig{elevatorPID}, 
    .range{.minPosition = 0.0_tr, .maxPosition = maxTurns, .continuousWrap = true}, 
    .feedbackConfig{.sensorToMechanismRatio = 0.0 /*GearRatio*/ },
    .openLoopConfig{}, 
    .currentLimits{}
};
const rmb::TalonFXPositionController::CreateInfo elevatorControllerCreateInfo2{
    .config{.id = 62, .inverted = false, .brake = true}, 
    .pidConfig{elevatorPID}, 
    .range{.minPosition = 0.0_tr, .maxPosition = maxTurns, .continuousWrap = true}, 
    .feedbackConfig{.sensorToMechanismRatio = 0.0 /*GearRatio*/ },
    .openLoopConfig{}, 
    .currentLimits{}
};

}

namespace wrist {
    const double wrist_kG = 0.0; 
    const rmb::SparkMaxPositionController::CreateInfo wristPositionControllerCreateInfo{
        .motorConfig{
            .id = 51, 
            .motorType = rev::spark::SparkMax::MotorType::kBrushless,
            .inverted = false
        },
        .pidConfig{
            .p = 0.0, 
            .i = 0.0, 
            .d = 0.0, 
            .ff = 0.0, 
            .tolerance = 0.0_rad, 
            .iZone = 0.0, 
            .iMaxAccumulator = 0.0, 
            .maxOutput = 0.0, 
            .minOutput = 0.0
        },

        .range{
            .minPosition =0.0_tr, 
            .maxPosition = 0.0_tr,
            .isContinuous = false
        },
        .profileConfig{},
        .feedbackConfig{
            .gearRatio = 9,

        },
        .followers{},
        
    };

}
}