#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <numbers>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include "Constants.h"

class swerveModule {
 public:
  swerveModule(const double module[]);

  enum class ConfigType {motorDrive, motorTurn, encoderTurn};
  //void ConfigModule(const ConfigType& type);

  // frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();
  frc::SwerveModuleState GetState();
  void SetDesiredState(const frc::SwerveModuleState& state);

  frc::SwerveModuleState CustomOptimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle);

  enum class DataType {kCurrentAngle, kCurrentVelocity, kTargetAngle};
  double DashboardInfo(const DataType& type);

 private:
  //ctre::phoenix::motorcontrol::can::WPI_TalonFX m_motorDrive;
  //ctre::phoenix::motorcontrol::can::WPI_TalonFX m_motorTurn;
  
  ctre::phoenix6::hardware::CANcoder m_encoderTurn;
  rev::CANSparkMax m_motorDrive;
  rev::CANSparkMax m_motorTurn;
  rev::SparkRelativeEncoder m_encoderDrive = m_motorDrive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
  rev::SparkRelativeEncoder m_encoderTurnIntegrated = m_motorTurn.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

  rev::SparkPIDController m_driveController = m_motorDrive.GetPIDController();
  rev::SparkPIDController m_turnController = m_motorTurn.GetPIDController();

  // const double m_encoderOffset;
  double m_targetAngle;

  // hardwareSettings m_settings;
};

namespace units {
  UNIT_ADD(angle, native_unit, native_units, nu, unit<std::ratio<360, 2048>, units::degrees>) // 2048 clicks per rotation.
  UNIT_ADD(angular_velocity, native_units_per_decisecond, native_units_per_decisecond, nu_per_ds,
           compound_unit<native_units, inverse<deciseconds>>) // clicks per 100ms (standard FX output).
  //UNIT_ADD(angle, drive_gearing, drive_gearing, dratio, unit<std::ratio<27, 4>, units::degrees>)
  UNIT_ADD(length, wheel_circumference, wheel_circumferences, wcrc, unit<std::ratio<32, 100>, units::meters>) // 4 in diameter.
}
