#pragma once

#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include "swerveModule.h"
#include "Constants.h"

#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>

class drivetrain : public frc2::SubsystemBase {
 public:
  drivetrain();

  void SwerveDrive(units::meters_per_second_t xSpeed,
                   units::meters_per_second_t ySpeed,
                   units::radians_per_second_t zRot,
                   bool fieldRelative);

  void UpdateOdometry();
  void AddDataFromVision();
  void resetGyro();
  frc::Pose2d GetOdometry();
  void ResetOdometry(frc::Pose2d initPose);
  void ResetOdometryNot180(frc::Pose2d initPose);
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void slowDown();
  void normalSpeed();

  double kslowConst = -1.0;

private:
  
  // navX
  AHRS m_navX{frc::SerialPort::kMXP};
  
  // Swervedrive dimensions
  frc::Translation2d m_locationFrontRight{+14_in, -14_in};
  frc::Translation2d m_locationRearRight{-14_in, -14_in};
  frc::Translation2d m_locationFrontLeft{+14_in, +14_in};
  frc::Translation2d m_locationRearLeft{-14_in, +14_in};

  // Creates new objects for each swerve module
  swerveModule m_frontRight{drivetrainConstants::swerveModules::kModuleFrontRight};
  swerveModule m_rearRight{drivetrainConstants::swerveModules::kModuleRearRight};
  swerveModule m_frontLeft{drivetrainConstants::swerveModules::kModuleFrontLeft};
  swerveModule m_rearLeft{drivetrainConstants::swerveModules::kModuleRearLeft};

  // Creates SwerveDrive Kinematics object
  frc::SwerveDriveKinematics<4> m_kinematics{m_locationFrontRight,
                                             m_locationRearRight,
                                             m_locationFrontLeft,
                                             m_locationRearLeft};

  // Creates SwerveDrive Odometry object
  frc::SwerveDrivePoseEstimator<4> m_odometry{m_kinematics, m_navX.GetRotation2d(), {m_frontRight.GetPosition(), m_frontLeft.GetPosition(), m_rearRight.GetPosition(), m_rearLeft.GetPosition()}, frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))};
};
