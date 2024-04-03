// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include <units/length.h>
#include <rev/CANSparkMax.h>
#include <Constants.h>

using namespace climberConstants;

class NewClimber : public frc2::SubsystemBase {
 public:
  enum solenoidStates {
    LOCKED,
    UNLOCKED
  };
  NewClimber();
  // Set height of the climbers. if the solenoids are locked, the climbers will not move until they are unlocked.
  void SetHeight(units::length::meter_t leftHeight, units::length::meter_t rightHeight);
  void SetHeight(units::length::meter_t height);
  // Set wether the solenoids are locked
  void setSolenoidState(solenoidStates state);
  // Check if climbing is allowed
  bool isClimbAllowed();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
    rev::CANSparkMax m_climberMotorLeft;
    rev::CANSparkMax m_climberMotorRight;
    rev::SparkPIDController m_climberMotorLeftController = m_climberMotorLeft.GetPIDController();
    rev::SparkPIDController m_climberMotorRightController = m_climberMotorRight.GetPIDController();
    rev::SparkRelativeEncoder m_climberMotorLeftEncoder = m_climberMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder m_climberMotorRightEncoder = m_climberMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

    rev::CANSparkMax m_climberSolenoidLeft;
    rev::CANSparkMax m_climberSolenoidRight;

    units::length::meter_t m_leftDesiredHeight;
    units::length::meter_t m_rightDesiredHeight;
    solenoidStates m_solenoidState = solenoidStates::LOCKED;
    units::time::second_t m_unlockTime;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
