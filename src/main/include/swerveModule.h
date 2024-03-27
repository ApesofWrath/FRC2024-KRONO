#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <numbers>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include "Constants.h"

class swerveModule {
 public:
  swerveModule(const double module[]);

  enum class ConfigType {motorDrive, motorTurn, encoderTurn};

  frc::SwerveModulePosition GetPosition();
  frc::SwerveModuleState GetState();
  void SetDesiredState(const frc::SwerveModuleState& state);

  frc::SwerveModuleState CustomOptimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle);

  enum class DataType {kCurrentAngle, kCurrentVelocity, kTargetAngle};
  double DashboardInfo(const DataType& type);

 private:
  
  ctre::phoenix6::hardware::CANcoder m_encoderTurn;
  rev::CANSparkMax m_motorDrive;
  rev::CANSparkMax m_motorTurn;
  rev::SparkRelativeEncoder m_encoderDrive = m_motorDrive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoderTurnIntegrated = m_motorTurn.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

  rev::SparkPIDController m_driveController = m_motorDrive.GetPIDController();
  rev::SparkPIDController m_turnController = m_motorTurn.GetPIDController();

  double m_targetAngle;
};