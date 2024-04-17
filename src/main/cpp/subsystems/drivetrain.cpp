
#include "subsystems/drivetrain.h"
using namespace drivetrainConstants::calculations;

// Constructor, zeros the gyro for swervedrive
drivetrain::drivetrain() 
:  m_pigeon(drivetrainConstants::kPigeonID)
{
    m_pigeon.Reset();
    printf("Drive Constructor");
    using namespace pathplanner;
    AutoBuilder::configureHolonomic(
        [this](){ return GetOdometry(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ SwerveDrive(speeds.vx, speeds.vy, speeds.omega, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            units::meters_per_second_t(kModuleMaxSpeed), // Max module speed, in m/s
            units::meter_t(kDrivebaseRadius), // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

void drivetrain::SwerveDrive(units::meters_per_second_t xSpeed,
                             units::meters_per_second_t ySpeed,
                             units::radians_per_second_t zRot,
                             bool fieldRelative) {
                            frc::ChassisSpeeds chassisSpeeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, zRot, m_pigeon.GetRotation2d())
                      : frc::ChassisSpeeds{xSpeed, ySpeed, zRot};
    chassisSpeeds = frc::ChassisSpeeds::Discretize(chassisSpeeds, 0.2_s);
    auto moduleStates = m_kinematics.ToSwerveModuleStates(chassisSpeeds);

    m_kinematics.DesaturateWheelSpeeds(
    &moduleStates,
    chassisSpeeds,
    kModuleMaxSpeed,
    kModuleMaxSpeed,
    kModuleMaxAngularVelocity
    );
    
    auto [frontRight, rearRight, frontLeft, rearLeft] = moduleStates;

    m_frontRight.SetDesiredState(frontRight);
    m_rearRight.SetDesiredState(rearRight);
    m_frontLeft.SetDesiredState(frontLeft);
    m_rearLeft.SetDesiredState(rearLeft);
}

// Updates the odometry of the swervedrive
void drivetrain::UpdateOdometry() {
    m_odometry.Update(m_pigeon.GetRotation2d(), {m_frontRight.GetPosition(),
                      m_rearRight.GetPosition(), m_frontLeft.GetPosition(),
                      m_rearLeft.GetPosition()});
}


// Perodically (Constantly runs during periodic), updates the odometry of the swervedrive
frc::Pose2d drivetrain::GetOdometry() {
    return m_odometry.GetEstimatedPosition();
}

frc::ChassisSpeeds drivetrain::GetRobotRelativeSpeeds(){
    return m_kinematics.ToChassisSpeeds(m_frontRight.GetState(), m_rearRight.GetState(), m_frontLeft.GetState(), m_rearLeft.GetState());
}

void drivetrain::ResetOdometry(frc::Pose2d initPose) {
    m_odometry.ResetPosition(m_pigeon.GetRotation2d(), {m_frontRight.GetPosition(),
                      m_rearRight.GetPosition(), m_frontLeft.GetPosition(),
                      m_rearLeft.GetPosition()}, initPose);
}

void drivetrain::ResetOdometryAngle(frc::Pose2d initPose, units::degree_t angle) {
    initPose.TransformBy(frc::Transform2d(frc::Translation2d(0_m, 0_m), frc::Rotation2d(angle)));
    m_odometry.ResetPosition(m_pigeon.GetRotation2d(), {m_frontRight.GetPosition(),
                      m_rearRight.GetPosition(), m_frontLeft.GetPosition(),
                      m_rearLeft.GetPosition()}, initPose);
    
}
void drivetrain::xStance() {
    m_frontRight.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(45.0_deg)));
    m_rearRight.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(-45.0_deg)));
    m_frontLeft.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(-45.0_deg)));
    m_rearLeft.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(45.0_deg)));
}

frc2::CommandPtr drivetrain::driveCommand(double xSpeed, double ySpeed, double zRotation) {
    return this->Run([this, xSpeed, ySpeed, zRotation] { SwerveDrive( // make m_drivetrain point to SwerveDrive function while passing the below values
                            -m_xSpeedLimiter.Calculate(frc::ApplyDeadband((xSpeed * m_speedMultiplier), 0.08)) * kModuleMaxSpeed, // ??
                            -m_ySpeedLimiter.Calculate(frc::ApplyDeadband((ySpeed * m_speedMultiplier), 0.08)) * kModuleMaxSpeed, // ??
                            -m_zRotationLimiter.Calculate(frc::ApplyDeadband((zRotation * m_speedMultiplier), 0.20)) * kModuleMaxAngularVelocity, 
                            true); });
}

// Resets the gyro when function run
frc2::CommandPtr drivetrain::resetGyroCommand() {
    return this->RunOnce([this] { m_pigeon.Reset(); });
}

// Slow constant value
frc2::CommandPtr drivetrain::slowDownCommand() {
    printf("Slow Func");
    return this->RunOnce([this] { m_speedMultiplier = 0.5; });
}

// Normal speed value (should always be 1.0)
frc2::CommandPtr drivetrain::normalSpeedCommand() {
    printf("Normal Func");
    return this->RunOnce([this] { m_speedMultiplier = 1.0; });
}

frc2::CommandPtr drivetrain::xStanceCommand() {
    return this->RunOnce([this] { xStance(); });
}

void drivetrain::Periodic() {
    UpdateOdometry(); 
}

void drivetrain::SimulationPeriodic() {}
