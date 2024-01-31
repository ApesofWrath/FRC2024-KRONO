// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


RobotContainer::RobotContainer() {

  // Initialize all of your commands and subsystems here
  pathplanner::NamedCommands::registerCommand("test", std::make_shared<frc2::PrintCommand>("This works :3"));
  // Configure the button bindings
  ConfigureButtonBindings();

  // $ CONTROLLER INPUTS FOR SWERVE DRIVE BELOW
  m_drivetrain.SetDefaultCommand(Drive(
    &m_drivetrain,
    [this] { return (MathFunctions::joystickCurve((m_controllerMain.GetX()), controllerConstants::kControllerCurve)); },
    [this] { return (MathFunctions::joystickCurve((m_controllerMain.GetY()), controllerConstants::kControllerCurve)); },
    [this] { return (-m_controllerMain.GetRawAxis(4)); }));

    m_chooser.SetDefaultOption("DoNothing", "DoNothing");
    m_chooser.AddOption("3Note", "3Note");
    m_chooser.AddOption("4Note", "4Note");
    m_chooser.AddOption("test", "TestAuto");

    frc::SmartDashboard::PutData(&m_chooser);
    
}

// All the button commands are set in this function
void RobotContainer::ConfigureButtonBindings() {

  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kA).OnTrue(IntakeToggle(&m_intake).ToPtr());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kB).OnTrue(ShooterToggle(&m_shooter).ToPtr());
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


