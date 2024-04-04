// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


using namespace generalConstants;
RobotContainer::RobotContainer() {

  // Initialize all of your commands and subsystems here
  pathplanner::NamedCommands::registerCommand("test", std::move(frc2::PrintCommand("hi :3").ToPtr()));
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
    m_chooser.AddOption("New Auto", "New Auto");

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


