// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/Drivetrain/ZeroGyro.h"
#include "commands/Drivetrain/NormalSpeed.h"
#include "commands/Drivetrain/SlowDown.h"

RobotContainer::RobotContainer() {

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // $ CONTROLLER INPUTS FOR SWERVE DRIVE BELOW
  m_drivetrain.SetDefaultCommand(Drive(
    &m_drivetrain,
    [this] { return (MathFunctions::joystickCurve((m_controllerMain.GetX()), controllerConstants::kControllerCurve)); },
    [this] { return (MathFunctions::joystickCurve((m_controllerMain.GetY()), controllerConstants::kControllerCurve)); },
    [this] { return (-m_controllerMain.GetRawAxis(4)); }));
    
}

// All the button commands are set in this function
void RobotContainer::ConfigureButtonBindings() {
  //** For commented command file, look at:
  //** ArmUp.cpp and
  //** ArmUp.h

  //Bind Limelight Pipeline 0 (Apriltags) and Pipeline 1 (Retroreflective) to ButtonA and ButtonB events respectively
    // frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kA).WhileTrue(frc2::InstantCommand([this] { m_vision.SelectPipeline(0); }).ToPtr());
  // frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kB).WhileTrue(frc2::InstantCommand([this] { m_vision.SelectPipeline(1); }).ToPtr());
  frc2::JoystickButton(&m_controllerMain, frc::XboxController::Button::kA).OnTrue(IntakeToggle(&m_intake).ToPtr());
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
  return PathPlannerAuto("Example Auto").ToPtr();
}


