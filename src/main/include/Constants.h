#pragma once
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <numbers>
    
/**
 * This header contains hold robot-wide numerical or boolean constants ONLY.
 * 
 * Place constants into subsystem/command -specific NAMESPACES within this
 * header, which can then be included (where they are needed).
 */
namespace generalConstants {
    constexpr double kRotationsToDegrees = 360.0;
}
namespace visionConstants {
    //How fast the robot attempts to correct its self
    constexpr double errorMultiplier = -0.025;
    //The minumum error for the robot to move
    constexpr double errorMinimum = 0.0;

    constexpr double xAlignMultiplier = 0.5;
    constexpr double yAlignMultiplier = 0;
    constexpr double yawAlignMultiplier = 0.0;
    // Robots distance from the April Tag it will align its self with
    constexpr double AlignDistance = 10.0;
    constexpr double rotationDistance = 1.0;
}

// IDs for controllers
namespace controllerConstants {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
    // Curve amount for drivers controller
    constexpr double kControllerCurve = 1.0;
}

namespace intakeConstants {
    constexpr int kIntakeMotorLeft = 13;
    constexpr int kIntakeMotorRight = 14;
    constexpr int kIntakeRotationMotor = 15;
    const auto OFF_SPEED = 0.0_tps;
    const auto IDLE_SPEED = 2.5_tps;
    const auto ON_SPEED = 10.0_tps;
    constexpr int kBeambreakCanifier = 18;
}

namespace shooterConstants {
    constexpr int kMotorShooterLeft = 16;
    constexpr int kMotorShooterRight = 17;
}

namespace climberConstants {
    constexpr int kMotorClimberLeft = 18;
    constexpr int kMotorClimberRight = 19;
    constexpr double kTelescopingRatio = 1 / 20;
    constexpr double kRotationsToInchTelescoping = 0.75 * std::numbers::pi;
    constexpr double kClimberUnextendedHeight = 0.0;
    constexpr double kClimberExtendedHeight = 2.0;
    constexpr int kSolenoidClimberLeft = 20;
    constexpr int kSolenoidClimberRight = 21;
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
        constexpr units::length::meter_t kWheelCircumference{2 * std::numbers::pi * kWheelDiameter};
        constexpr auto kModuleMaxSpeed{15.1_fps};
        constexpr auto kChassisMaxSpeed{15.1_fps};

        constexpr auto kModuleMaxAngularVelocity{std::numbers::pi * 2_rad_per_s};  // radians per second
        constexpr auto kModuleMaxAngularAcceleration{std::numbers::pi * 8_rad_per_s / 1_s};  // radians per second^2

        constexpr double kMotorMaxOutput = 0.5;
        constexpr double kMotorDeadband = 0.1;
    }
}
