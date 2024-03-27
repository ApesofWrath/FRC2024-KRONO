// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooterintake/AutoAngle.h"

AutoAngle::AutoAngle(intakeshooter* intakeshooter, vision* vision) : m_intakeshooter{intakeshooter}, m_vision{vision} {
  AddRequirements({intakeshooter});

  // Add points for interpolating map
  m_interpolatingMap.insert(52.1, 114.5);
  m_interpolatingMap.insert(64.5, 110.5);
  m_interpolatingMap.insert(75.5, 108);
  m_interpolatingMap.insert(89.9, 105);
  m_interpolatingMap.insert(108.3, 102);
  m_interpolatingMap.insert(120.9, 100);

}


// Called when the command is initially scheduled.
void AutoAngle::Initialize() {printf("AutoAngle Initialized");
  double distance = m_vision->getDistance();
  frc::SmartDashboard::PutNumber("Apriltag distance", distance);
  double angle{m_interpolatingMap[distance]};
  std::clamp(angle, 0.0, 127.0);
  m_intakeshooter->spinup(angle);
}

// Called repeatedly when this Command is scheduled to run
void AutoAngle::Execute() {}

// Called once the command ends or is interrupted.
void AutoAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAngle::IsFinished() {
  return false;
}
