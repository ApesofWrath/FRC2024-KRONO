// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include "RobotContainer.h"
#include "subsystems/drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/XboxController.h>
#include <commands/Drivetrain/Drive.h>
#include <frc/Joystick.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  RobotContainer m_container;
  drivetrain m_drivetrain;
};
