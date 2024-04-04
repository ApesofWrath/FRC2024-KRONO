#pragma once
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <numbers>
#include <frc/Timer.h>
    
/**
 * This header contains hold robot-wide numerical or boolean constants ONLY.
 * 
 * Place constants into subsystem/command -specific NAMESPACES within this
 * header, which can then be included (where they are needed).
 */
namespace generalConstants {
    constexpr double kRotationsToDegrees = 360.0;
    
    // Declare Global Timer Object
    // To use, include <frc/Timer.h>
    extern frc::Timer timer; 
}

// IDs for controllers
namespace controllerConstants {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerCmdID = 1;

    // Curve amount for drivers controller
    constexpr double kControllerCurve = 1.0;
}

// Motor IDs, Encoder IDs, and Offsets for swervedrive
namespace drivetrainConstants {
    //CAN IDs
    constexpr int kMotorDriveFrontRightID = 1;
    constexpr int kMotorDriveRearRightID = 3;
    constexpr int kMotorDriveFrontLeftID = 5;
    constexpr int kMotorDriveRearLeftID = 7;

    constexpr int kMotorTurnFrontRightID = 2;
    constexpr int kMotorTurnRearRightID = 4;
    constexpr int kMotorTurnFrontLeftID = 6;
    constexpr int kMotorTurnRearLeftID = 8;

    constexpr int kEncoderTurnFrontRightID = 9;
    constexpr int kEncoderTurnRearRightID = 10;
    constexpr int kEncoderTurnFrontLeftID = 11;
    constexpr int kEncoderTurnRearLeftID = 12;

    constexpr int kPigeonID = 13;

    // Values for each swerve module object to use
    namespace swerveModules {
        constexpr double kModuleFrontRight[3]{kMotorDriveFrontRightID,
                                                   kMotorTurnFrontRightID,
                                                   kEncoderTurnFrontRightID};
        constexpr double kModuleRearRight[3]{kMotorDriveRearRightID,
                                                  kMotorTurnRearRightID,
                                                  kEncoderTurnRearRightID};
        constexpr double kModuleFrontLeft[3]{kMotorDriveFrontLeftID,
                                                  kMotorTurnFrontLeftID,
                                                  kEncoderTurnFrontLeftID};
        constexpr double kModuleRearLeft[3]{kMotorDriveRearLeftID,
                                                 kMotorTurnRearLeftID,
                                                 kEncoderTurnRearLeftID};
    }

    // Math constants and calculations to be used by swervedrive
    namespace calculations {
        constexpr auto kFinalDriveRatio{(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)};
        constexpr auto kFinalTurnRatio{1.0 / (150.0 / 7.0)};
        constexpr units::length::meter_t kWheelDiameter{4.0_in};
        constexpr units::length::meter_t kWheelRadius{4.0_in / 2.0};
        constexpr units::length::meter_t kWheelCircumference{2 * std::numbers::pi * (kWheelDiameter / 2)};
        constexpr auto kModuleMaxSpeed{15.1_fps};
        constexpr auto kChassisMaxSpeed{15.1_fps};

        constexpr auto kModuleMaxAngularVelocity{12.957_rad_per_s};  // radians per second
    }
}
