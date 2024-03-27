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
#include <ctre/phoenix/CANifier.h>
#include <commands/Drivetrain/Align.h>
#include <subsystems/vision.h>

#include "Constants.h"
#include "commands/Drivetrain/Drive.h"
#include "commands/Drivetrain/ZeroGyro.h"
#include "commands/Drivetrain/NormalSpeed.h"
#include "commands/Drivetrain/SlowDown.h"

#include "commands/ShooterIntake/fire.h"
#include "commands/ShooterIntake/intakeActivate.h"
#include "commands/ShooterIntake/intakeRetract.h"
#include "commands/ShooterIntake/rapidFire.h"
#include "commands/ShooterIntake/scoreAmp.h"
#include "commands/ShooterIntake/spinup.h"

#include "subsystems/drivetrain.h"
#include "subsystems/intakeshooter.h"
#include "subsystems/LED.h"`

#include "MathFunctions.h"

#include "commands/Climber/ExtendClimber.h"
#include "commands/Climber/RetractClimber.h"
#include "commands/Climber/LeftClimbToggle.h"
#include "commands/Climber/RightClimbToggle.h"

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/Timer.h>

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
  ctre::phoenix::CANifier BeambreakLEDCanifier{intakeConstants::kBeambreakCanifier};

  drivetrain m_drivetrain;
  intakeshooter m_intakeshooter{&m_controllerMain, &m_controllerOperator, BeambreakLEDCanifier};
  climber m_climber;
  LED m_LED{BeambreakLEDCanifier};
  LEDmanager m_LEDmanager{m_LED, m_intakeshooter};
  vision m_vision;

  frc::SendableChooser<std::string> m_chooser;
  void ConfigureButtonBindings();

  // Controller creation
  frc2::CommandXboxController m_controllerMain{controllerConstants::kControllerMainID};
  frc2::CommandXboxController m_controllerOperator{controllerConstants::kControllerCmdID};
  frc2::CommandXboxController m_controllerAlt{controllerConstants::kControllerAltID};
};
