// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <subsystems/drivetrain.h>
#include <subsystems/vision.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Align
    : public frc2::CommandHelper<frc2::Command, Align> {
 public:
  Align(vision* vision, drivetrain* drivetrain);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    vision* m_vision;
    drivetrain* m_drivetrain;
};
