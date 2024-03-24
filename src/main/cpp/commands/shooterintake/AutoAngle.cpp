// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooterintake/AutoAngle.h"

AutoAngle::AutoAngle(intakeshooter* intakeshooter, vision* vision) : m_intakeshooter{intakeshooter}, m_vision{vision} {
  AddRequirements({intakeshooter});
  m_interpolatingMap.insert(0.0, 0.0);
}


// Called when the command is initially scheduled.
void AutoAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoAngle::Execute() {
}

// Called once the command ends or is interrupted.
void AutoAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAngle::IsFinished() {
  return false;
}
