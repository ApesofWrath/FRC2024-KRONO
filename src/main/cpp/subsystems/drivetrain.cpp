
#include "subsystems/drivetrain.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

// Constructor, zeros the gyro for swervedrive
drivetrain::drivetrain() {
    m_navX.ZeroYaw();
    printf("Drive Constructor");
    using namespace pathplanner;
    AutoBuilder::configureHolonomic(
        [this](){ return GetOdometry(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ SwerveDrive(speeds.vx, speeds.vy, speeds.omega, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
            PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
            units::meters_per_second_t(4.5), // Max module speed, in m/s
            units::meter_t(13.625_in), // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            return alliance.value() == frc::DriverStation::Alliance::kRed;
        },
        this // Reference to this subsystem to set requirements
    );
}

// Resets the gyro when function run
void drivetrain::resetGyro() {
    m_navX.ZeroYaw();
}

frc2::CommandPtr drivetrain::resetGyroCommand() { // return pointer to setter command w/ AndThen for the command's IsFinished
    return frc2::Subsystem::RunOnce([this]() {this->resetGyro();}).AndThen(frc2::WaitUntilCommand([this]() {return true;}).ToPtr());
}

// Slow constant value
void drivetrain::slowDown() {
    kslowConst = 0.5;
    printf("Slow Func");
}

frc2::CommandPtr drivetrain::slowDownCommand() { // return pointer to setter command w/ AndThen for the command's IsFinished
    return frc2::Subsystem::RunOnce([this]() {this->slowDown();}).AndThen(frc2::WaitUntilCommand([this]() {return true;}).ToPtr());
}

// Normal speed value (should always be 1.0)
void drivetrain::normalSpeed() {
    kslowConst = 1.0;
    printf("Normal Func");
}

frc2::CommandPtr drivetrain::normalSpeedCommand() { // return pointer to setter command w/ AndThen for the command's IsFinished
    return frc2::Subsystem::RunOnce([this]() {this->normalSpeed();}).AndThen(frc2::WaitUntilCommand([this]() {return true;}).ToPtr());
}

// Sets Desired States of the swerve modules for swervedrive
void drivetrain::SwerveDrive(units::meters_per_second_t xSpeed,
                             units::meters_per_second_t ySpeed,
                             units::radians_per_second_t zRot,
                             bool fieldRelative) {
                            frc::ChassisSpeeds chassisSpeeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, zRot, m_navX.GetRotation2d())
                      : frc::ChassisSpeeds{xSpeed, ySpeed, zRot};
    auto moduleStates = m_kinematics.ToSwerveModuleStates(chassisSpeeds);
                      
    m_kinematics.DesaturateWheelSpeeds(
    &moduleStates,
    chassisSpeeds,
    drivetrainConstants::calculations::kModuleMaxSpeed,
    drivetrainConstants::calculations::kModuleMaxSpeed,
    drivetrainConstants::calculations::kModuleMaxAngularVelocity
    );

    frc::SmartDashboard::PutNumber("zRotation", zRot.value());
    frc::SmartDashboard::PutNumber("zRotation actual", m_navX.GetRate() * -1.0);
    frc::SmartDashboard::PutNumber("Robot Rotation", m_navX.GetRotation2d().Degrees().value());
    frc::SmartDashboard::PutNumber("navX Yaw", m_navX.GetYaw());
    frc::SmartDashboard::PutNumber("Od Rob Rot", m_odometry.GetEstimatedPosition().Rotation().Degrees().value());
    
    auto [frontRight, rearRight, frontLeft, rearLeft] = moduleStates;

    m_frontRight.SetDesiredState(frontRight);
    m_rearRight.SetDesiredState(rearRight);
    m_frontLeft.SetDesiredState(frontLeft);
    m_rearLeft.SetDesiredState(rearLeft);
}

frc2::CommandPtr drivetrain::SwerveDriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed, std::function<double()> zRotation) { // oh lird
    return frc2::Subsystem::RunOnce([this, xSpeed, ySpeed, zRotation]() {this->SwerveDrive(-m_ySpeedLimiter.Calculate(frc::ApplyDeadband((m_ySpeed() * kslowConst), 0.08)) * drivetrainConstants::calculations::kChassisMaxSpeed, -m_xSpeedLimiter.Calculate(frc::ApplyDeadband((m_xSpeed() * kslowConst), 0.08)) * drivetrainConstants::calculations::kChassisMaxSpeed, -m_zRotationLimiter.Calculate(frc::ApplyDeadband((m_zRotation() * kslowConst), 0.20)) * drivetrainConstants::calculations::kModuleMaxAngularVelocity, true);}).AndThen(frc2::WaitUntilCommand([this]() {return false;}).ToPtr());
}

// Updates the odometry of the swervedrive
void drivetrain::UpdateOdometry() {
    m_odometry.Update(m_navX.GetRotation2d(), {m_frontRight.GetPosition(),
                      m_rearRight.GetPosition(), m_frontLeft.GetPosition(),
                      m_rearLeft.GetPosition()});
}

// Perodically (Constantly runs during periodic), updates the odometry of the swervedrive
frc::Pose2d drivetrain::GetOdometry() {
    return m_odometry.GetEstimatedPosition();
}

void drivetrain::ResetOdometry180(frc::Pose2d initPose) {
    initPose.TransformBy(frc::Transform2d(frc::Translation2d(0_m, 0_m), frc::Rotation2d(180_deg)));
    m_odometry.ResetPosition(m_navX.GetRotation2d(), {m_frontRight.GetPosition(),
                      m_rearRight.GetPosition(), m_frontLeft.GetPosition(),
                      m_rearLeft.GetPosition()}, initPose);
    
}

void drivetrain::ResetOdometry(frc::Pose2d initPose) {
    m_odometry.ResetPosition(m_navX.GetRotation2d(), {m_frontRight.GetPosition(),
                      m_rearRight.GetPosition(), m_frontLeft.GetPosition(),
                      m_rearLeft.GetPosition()}, initPose);
}

frc::ChassisSpeeds drivetrain::GetRobotRelativeSpeeds(){
    frc::ChassisSpeeds speeds = m_kinematics.ToChassisSpeeds(m_frontRight.GetState(), m_rearRight.GetState(), m_frontLeft.GetState(), m_rearLeft.GetState());
    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds.vx, speeds.vy, speeds.omega, m_navX.GetRotation2d());
}

void drivetrain::Periodic() {
    UpdateOdometry();
    frc::SmartDashboard::PutNumber("Odometry X", units::unit_cast<double>(m_odometry.GetEstimatedPosition().X()));
    frc::SmartDashboard::PutNumber("Odometry Y", units::unit_cast<double>(m_odometry.GetEstimatedPosition().Translation().Y()));
    frc::SmartDashboard::PutNumber("Odometry Rot", units::unit_cast<double>(m_odometry.GetEstimatedPosition().Rotation().Degrees()));
}

void drivetrain::SimulationPeriodic() {}
