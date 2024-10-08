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
namespace visionConstants {
    //How fast the robot attempts to correct its self
    constexpr double Kp = -0.1;
    constexpr double min_command = 0.05;
}

// IDs for controllers
namespace controllerConstants {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerCmdID = 1;

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
    constexpr int kIntakeAngleTolerance = 1;
    constexpr int kPigeon = 22;

    constexpr double kIntakeResetAngle = 0; // 0 for encoder; -66.138 pigeon
    constexpr double kIntakeSpeakerAngle = 116.5; // 114.5 for encoder; 56.42 pigeon
    constexpr double kIntakeAmpAngle = 23; // 26 for encoder; -51.24 pigeon
    constexpr double kIntakeIntakingAngle = 126; // 126 for encoder; 61.34 pigeon

    /* 246.138 - 180 + (-66.138) = 0
    246.138 - 180 + (56.42) =   */
}

namespace shooterConstants {
    constexpr int kMotorShooterLeft = 16;
    constexpr int kMotorShooterRight = 17;
    constexpr double kShooterRPMTolerance = 300;
	constexpr bool usePidgeonAlways = false; // true = mbr config (slips); false = master config (wiggles)
}

namespace climberConstants {
    constexpr int kMotorClimberLeft = 18;
    constexpr int kMotorClimberRight = 19;
    constexpr double kTelescopingRatio = 1.0 / 16.0;
    constexpr double kRotationsToInchTelescoping = 1.05 * std::numbers::pi;
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
        constexpr units::length::meter_t kWheelCircumference{2 * std::numbers::pi * (kWheelDiameter / 2)};
        constexpr auto kModuleMaxSpeed{15.1_fps};
        constexpr auto kChassisMaxSpeed{15.1_fps};

        constexpr auto kModuleMaxAngularVelocity{12.957_rad_per_s};  // radians per second
        constexpr auto kModuleMaxAngularAcceleration{std::numbers::pi * 8_rad_per_s / 1_s};  // radians per second^2

        constexpr double kMotorMaxOutput = 0.5;
        constexpr double kMotorDeadband = 0.1;
    }
}
