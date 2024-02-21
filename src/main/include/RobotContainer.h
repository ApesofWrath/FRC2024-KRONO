// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/Command/Button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"
#include "commands/Drivetrain/Drive.h"
#include "commands/Drivetrain/ZeroGyro.h"
#include "commands/Drivetrain/NormalSpeed.h"
#include "commands/Drivetrain/SlowDown.h"
#include "commands/Drivetrain/Allign.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intakeshooter.h"
#include "MathFunctions.h"
#include "commands/SetClimber.h"
#include "subsystems/vision.h"

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  
  frc2::CommandPtr GetAutonomousCommand();
 private:
  // The robot's subsystems and commands are defined here...
  drivetrain m_drivetrain;
  intakeshooter m_intakeshooter;
  vision m_vision;

  frc::SendableChooser<std::string> m_chooser;
  void ConfigureButtonBindings();

  // Controller creation
  frc::Joystick m_controllerMain{controllerConstants::kControllerMainID};
  frc2::CommandXboxController m_controllerOperator{controllerConstants::kControllerAuxID};
};
