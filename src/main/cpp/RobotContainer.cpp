// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {

  // Initialize all of your commands and subsystems here
  pathplanner::NamedCommands::registerCommand("spinup", std::move(m_intakeshooter.spinupCommand(110.0)));
  pathplanner::NamedCommands::registerCommand("fire", std::move(m_intakeshooter.fireCommand()));
  pathplanner::NamedCommands::registerCommand("intakeActivate", std::move(m_intakeshooter.intakeActivateCommand()));
  pathplanner::NamedCommands::registerCommand("intakeRetract", std::move(m_intakeshooter.intakeRetractCommand()));
  pathplanner::NamedCommands::registerCommand("rapidFire", std::move(m_intakeshooter.rapidFireCommand()));
  // Configure the button bindings
  ConfigureButtonBindings();

  // $ CONTROLLER INPUTS FOR SWERVE DRIVE BELOW
  m_drivetrain.SetDefaultCommand(m_drivetrain.SwerveDriveCommand(
    [this] { return (MathFunctions::joystickCurve((m_controllerMain.GetLeftX()), controllerConstants::kControllerCurve)); },
    [this] { return (MathFunctions::joystickCurve((m_controllerMain.GetLeftY()), controllerConstants::kControllerCurve)); },
    [this] { return (m_controllerMain.GetRawAxis(4)); }));

    m_chooser.SetDefaultOption("DoNothing", "DoNothing");
    m_chooser.AddOption("2NoteCenter", "2NoteCenter");
    m_chooser.AddOption("2NoteAmpSide", "2NoteAmpSide");
    m_chooser.AddOption("3NoteAmpSide", "3NoteAmpSide");
    m_chooser.AddOption("3NoteCenter", "3NoteCenter");
    m_chooser.AddOption("4Note", "4Note");
    m_chooser.AddOption("Backup", "Backup");
    m_chooser.AddOption("Preload", "Preload");
    m_chooser.AddOption("PreloadBackupCenter", "PreloadBackupCenter");

    frc::SmartDashboard::PutData(&m_chooser);
    
}

// All the button commands are set in this function
void RobotContainer::ConfigureButtonBindings() {

  // Zeroing for swervedrive command
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kStart).OnTrue(m_drivetrain.resetGyroCommand());

  // Slow button for swerve (whenever left OR right bumper is held down), slows swerve to slow value
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kRightBumper).OnTrue(m_drivetrain.slowDownCommand());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kRightBumper).OnFalse(m_drivetrain.normalSpeedCommand());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kLeftBumper).OnTrue(m_drivetrain.slowDownCommand());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kLeftBumper).OnFalse(m_drivetrain.normalSpeedCommand());
  
  // ShooterIntake buttons
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kLeftBumper).OnTrue(m_intakeshooter.intakeActivateCommand());
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kB).OnTrue(m_intakeshooter.spinupCommand(111.5)); // spinup for far speaker shot (7 feet from speaker)
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kX).OnTrue(m_intakeshooter.spinupCommand(110.0)); // spinup for near speaker shot (right at speaker)
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kRightBumper).OnTrue(m_intakeshooter.fireCommand());
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kA).OnTrue(m_intakeshooter.intakeRetractCommand());
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kY).OnTrue(m_intakeshooter.scoreAmpCommand());
  
  // Climber Buttons
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kLeftStick).OnTrue(m_climber.climberExtendCommand());
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kRightStick).OnTrue(m_climber.climberRetractCommand());

  // Climber Zero Maintinence Buttons
  frc2::JoystickButton(&m_controllerAlt, frc::XboxController::Button::kLeftBumper).OnTrue(m_climber.leftClimbToggleCommand());
  frc2::JoystickButton(&m_controllerAlt, frc::XboxController::Button::kRightBumper).OnTrue(m_climber.rightClimbToggleCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  using namespace pathplanner;
  return m_chooser.GetSelected() == "DoNothing" ? frc2::WaitCommand(15_s).ToPtr() : PathPlannerAuto(m_chooser.GetSelected()).ToPtr();
}


