// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


using namespace generalConstants;
RobotContainer::RobotContainer() {

  // Initialize all of your commands and subsystems here
  pathplanner::NamedCommands::registerCommand("spinup", std::move(spinup(&m_intakeshooter, 110.0).ToPtr()));
  pathplanner::NamedCommands::registerCommand("fire", std::move(fire(&m_intakeshooter).ToPtr()));
  pathplanner::NamedCommands::registerCommand("intakeActivate", std::move(intakeActivate(&m_intakeshooter).ToPtr()));
  pathplanner::NamedCommands::registerCommand("intakeRetract", std::move(intakeRetract(&m_intakeshooter).ToPtr()));
  pathplanner::NamedCommands::registerCommand("rapidFire", std::move(rapidFire(&m_intakeshooter).ToPtr()));
  // Configure the button bindings
  ConfigureButtonBindings();

  // $ CONTROLLER INPUTS FOR SWERVE DRIVE BELOW
  m_drivetrain.SetDefaultCommand(Drive(
    &m_drivetrain,
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
    
    // Start Timer
    timer.Start();
  
}

// All the button commands are set in this function
void RobotContainer::ConfigureButtonBindings() {

  // Zeroing for swervedrive command
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kStart).OnTrue(ZeroGyro(&m_drivetrain).ToPtr());

  // Slow button for swerve (whenever left OR right bumper is held down), slows swerve to slow value
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kRightBumper).OnTrue(SlowDown(&m_drivetrain).ToPtr());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kRightBumper).OnFalse(NormalSpeed(&m_drivetrain).ToPtr());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kLeftBumper).OnTrue(SlowDown(&m_drivetrain).ToPtr());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kLeftBumper).OnFalse(NormalSpeed(&m_drivetrain).ToPtr());
  
  // ShooterIntake buttons
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kLeftBumper).OnTrue(intakeActivate(&m_intakeshooter).ToPtr()); // kA
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kB).OnTrue(spinup(&m_intakeshooter, 100.8).ToPtr()); // spinup for far speaker shot (7 feet from speaker) !!!!96.6 (111.5)
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kX).OnTrue(spinup(&m_intakeshooter, intakeConstants::kIntakeSpeakerAngle).ToPtr()); // spinup for near speaker shot (right at speaker) Y !!!!!110.0
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kRightBumper).OnTrue(fire(&m_intakeshooter).ToPtr());
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kA).OnTrue(intakeRetract(&m_intakeshooter).ToPtr()); //leftbumper
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kY).OnTrue(scoreAmp(&m_intakeshooter).ToPtr());
  
  // Climber Buttons
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kLeftStick).OnTrue(ExtendClimber(&m_climber).ToPtr());
  frc2::JoystickButton(&m_controllerOperator, frc::XboxController::Button::kRightStick).OnTrue(RetractClimber(&m_climber).ToPtr());

  // Climber Zero Maintinence Buttons
  frc2::JoystickButton(&m_controllerAlt, frc::XboxController::Button::kLeftBumper).OnTrue(LeftClimbToggle(&m_climber).ToPtr());
  frc2::JoystickButton(&m_controllerAlt, frc::XboxController::Button::kRightBumper).OnTrue(RightClimbToggle(&m_climber).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  using namespace pathplanner;
  if (m_chooser.GetSelected() == "DoNothing") {
    return frc2::WaitCommand(15_s).ToPtr();
  }
  else {
    return PathPlannerAuto(m_chooser.GetSelected()).ToPtr();
  }
  
}


