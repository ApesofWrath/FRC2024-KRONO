#pragma once

#include <numbers>
#include <iostream>
#include <numbers>
#include <thread>
#include <chrono>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class swerveModule {
 public:
  swerveModule(const double module[]);

  frc::SwerveModulePosition GetPosition();
  frc::SwerveModuleState GetState();
  void SetDesiredState(const frc::SwerveModuleState& state);

  frc::SwerveModuleState CustomOptimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle);

 private:
  

  ctre::phoenix6::hardware::TalonFX m_motorDrive;
  ctre::phoenix6::hardware::TalonFX m_motorTurn;
  ctre::phoenix6::hardware::CANcoder m_encoderTurn;

  ctre::phoenix6::configs::CurrentLimitsConfigs m_motorDriveCurrentLimitsConfigs{};
  ctre::phoenix6::configs::CurrentLimitsConfigs m_motorTurnCurrentLimitsConfigs{};

  ctre::phoenix6::configs::Slot0Configs m_motorDriveSlot0Configs{};
  ctre::phoenix6::configs::Slot0Configs m_motorTurnSlot0Configs{};
  
  ctre::phoenix6::configs::FeedbackConfigs m_motorDriveFeedbackConfigs{};
  ctre::phoenix6::configs::FeedbackConfigs m_motorTurnFeedbackConfigs{};

  ctre::phoenix6::controls::VelocityVoltage m_driveMotorSpeed{0.0_tps};
  ctre::phoenix6::controls::PositionVoltage m_turnMotorAngle{0.0_tr};
};
