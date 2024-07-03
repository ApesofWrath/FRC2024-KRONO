#include "subsystems/drivetrain.h"

// Constructor, zeros the gyro for swervedrive
drivetrain::drivetrain() {
    m_navX.ZeroYaw();
    printf("Drive Constructor");
    using namespace pathplanner;
    AutoBuilder::configureHolonomic(
        [this](){ return GetOdometry(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ DriveRobotRelativeSpeeds(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            units::meters_per_second_t(4.5), // Max module speed, in m/s
            units::meter_t(13.625_in), // Drive base radius in meters. Distance from robot center to furthest module.
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

    frc::SmartDashboard::PutNumber("zRotation", units::degrees_per_second_t(zRot).value());
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

// Updates the odometry of the swervedrive
frc2::CommandPtr drivetrain::UpdateOdometry() {
    return frc2::cmd::RunOnce([this]{ m_odometry.Update(m_navX.GetRotation2d(), {m_frontRight.GetPosition(),
		m_rearRight.GetPosition(), m_frontLeft.GetPosition(),
		m_rearLeft.GetPosition()});
	});
}

// Resets the gyro when function run
frc2::CommandPtr drivetrain::resetGyro() {
    return frc2::cmd::RunOnce([this]{ m_navX.ZeroYaw(); });
}

// Slow the constant value until the command is inturrupted
frc2::CommandPtr drivetrain::slowDown() {
    return frc2::cmd::RunOnce([this]{ kslowConst = 0.5; }).AndThen(frc2::cmd::Idle()).FinallyDo([this]{ kslowConst = 1.0; });
}

frc2::CommandPtr drivetrain::xStance() {
	return frc2::cmd::Run([this]{
		m_frontRight.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(45.0_deg)));
		m_rearRight.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(-45.0_deg)));
		m_frontLeft.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(-45.0_deg)));
		m_rearLeft.SetDesiredState(frc::SwerveModuleState(0.0_mps, frc::Rotation2d(45.0_deg)));
	});
}

frc2::CommandPtr drivetrain::squareUp(vision* vision) {
	return frc2::cmd::RunOnce([this, vision]{
		double heading_error = vision->getHeadingError();
		units::angular_velocity::radians_per_second_t heading_error_radians{heading_error};
		SwerveDrive(0.0_mps, 0.0_mps, heading_error_radians, false);
	});
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

// Perodically (Constantly runs during periodic), updates the odometry of the swervedrive
frc::Pose2d drivetrain::GetOdometry() {
    return m_odometry.GetEstimatedPosition();
}

void drivetrain::DriveRobotRelativeSpeeds(frc::ChassisSpeeds robotRelativeSpeeds) {
    frc::ChassisSpeeds targetSpeeds = frc::ChassisSpeeds::Discretize(robotRelativeSpeeds, 0.2_s);
    auto moduleStates = m_kinematics.ToSwerveModuleStates(targetSpeeds);
    auto [frontRight, rearRight, frontLeft, rearLeft] = moduleStates;

    m_frontRight.SetDesiredState(frontRight);
    m_rearRight.SetDesiredState(rearRight);
    m_frontLeft.SetDesiredState(frontLeft);
    m_rearLeft.SetDesiredState(rearLeft);
}

frc::ChassisSpeeds drivetrain::GetRobotRelativeSpeeds(){
    return m_kinematics.ToChassisSpeeds(m_frontRight.GetState(), m_rearRight.GetState(), m_frontLeft.GetState(), m_rearLeft.GetState());
}

void drivetrain::Periodic() {
    UpdateOdometry();
	frc::SmartDashboard::PutNumber("Slowness", kslowConst);
    frc::SmartDashboard::PutNumber("Odometry X", units::unit_cast<double>(m_odometry.GetEstimatedPosition().X()));
    frc::SmartDashboard::PutNumber("Odometry Y", units::unit_cast<double>(m_odometry.GetEstimatedPosition().Translation().Y()));
    frc::SmartDashboard::PutNumber("Odometry Rot", units::unit_cast<double>(m_odometry.GetEstimatedPosition().Rotation().Degrees()));
	frc::SmartDashboard::PutNumber("Current Angle fr", m_frontRight.DashboardInfo(swerveModule::DataType::kCurrentAngle));   
	frc::SmartDashboard::PutNumber("Current Angle fl", m_frontLeft.DashboardInfo(swerveModule::DataType::kCurrentAngle));   
	frc::SmartDashboard::PutNumber("Current Angle br", m_rearRight.DashboardInfo(swerveModule::DataType::kCurrentAngle));   
	frc::SmartDashboard::PutNumber("Current Angle bl", m_rearLeft.DashboardInfo(swerveModule::DataType::kCurrentAngle));   
	frc::SmartDashboard::PutNumber("Target Angle fr", m_frontRight.DashboardInfo(swerveModule::DataType::kTargetAngle));   
	frc::SmartDashboard::PutNumber("Target Angle fl", m_frontLeft.DashboardInfo(swerveModule::DataType::kTargetAngle));   
	frc::SmartDashboard::PutNumber("Target Angle br", m_rearRight.DashboardInfo(swerveModule::DataType::kTargetAngle));   
	frc::SmartDashboard::PutNumber("Target Angle bl", m_rearLeft.DashboardInfo(swerveModule::DataType::kTargetAngle));   
}

void drivetrain::SimulationPeriodic() {}