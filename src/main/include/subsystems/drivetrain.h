#pragma once

#include <AHRS.h>
#include <iostream>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <subsystems/vision.h>
#include "swerveModule.h"
#include "Constants.h"

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

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

  frc2::CommandPtr UpdateOdometry();
  frc2::CommandPtr resetGyro();
  frc2::CommandPtr slowDown();
  frc2::CommandPtr normalSpeed();
  frc2::CommandPtr xStance(); // assert dominance by being hard to move
  frc2::CommandPtr squareUp(vision* vision);
  void ResetOdometry180(frc::Pose2d initPose);
  void ResetOdometry(frc::Pose2d initPose);
  frc::Pose2d GetOdometry();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  void DriveRobotRelativeSpeeds(frc::ChassisSpeeds robotRelativeSpeeds);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;


  double kslowConst = 1.0;

private:
  
  // navX
  AHRS m_navX{frc::SPI::kMXP};

  // Swervedrive dimensions
  frc::Translation2d m_locationFrontRight{+9.875_in, -9.875_in}; //9.875
  frc::Translation2d m_locationRearRight{-9.875_in, -9.875_in};
  frc::Translation2d m_locationFrontLeft{+9.875_in, +9.875_in};
  frc::Translation2d m_locationRearLeft{-9.875_in, +9.875_in};

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
