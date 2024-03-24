// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drivetrain/Align.h"

Align::Align(vision* vision, drivetrain* drivetrain) : m_vision{vision}, m_drivetrain{drivetrain} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrain});
}

// Called when the command is initially scheduled.
void Align::Initialize() {printf("Align Initialized");}

// Called repeatedly when this Command is scheduled to run
void Align::Execute() {
  double heading_error = m_vision->getHeadingError();
  units::angular_velocity::radians_per_second_t heading_error_radians{heading_error};
  m_drivetrain->SwerveDrive(0.0_mps, 0.0_mps, heading_error_radians, false);
}

// Called once the command ends or is interrupted.
void Align::End(bool interrupted) {}

// Returns true when the command should end.
bool Align::IsFinished() {
  return false;
}
