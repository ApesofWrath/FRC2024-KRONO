#pragma once

#include <numbers>
#include <string>
#include <iostream>
#include <units/length.h>
#include <Constants.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

enum class intakeshooterStates { // proceed cyclically down list, each comment describes state & conditions for entering
    IDLE, // default state, wheels (except feeder) spinning slowly
    INTAKEING, // enter on intake button, intake angles down & spins wheels
    HOLDING, // enter on note at correct position (sensor), ensure note position correctness
    SPINUP, // enter on fire button press, firing wheels go to max speed
    FIRE // enter on firing wheels reaching max velocity, feed note to shooter wheels, go to idlewhen note gone
};

class intakeshooter : public frc2::SubsystemBase {
    public:
    intakeshooter();
    void IntakeToggle();

    void Periodic() override;
    private:
    ctre::phoenix6::hardware::TalonFX m_intakeMotorLeft;
    ctre::phoenix6::hardware::TalonFX m_intakeMotorRight;

    ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    ctre::phoenix6::configs::Slot0Configs slotZeroConfigs{};

    ctre::phoenix6::controls::VelocityDutyCycle m_velocity{0_tps};

    rev::CANSparkMax m_shooterMotorLeft;
    rev::CANSparkMax m_shooterMotorRight;
    rev::CANSparkMax m_rotationMotor;

    rev::CANPIDController m_shooterMotorLeftController = m_shooterMotorLeft.GetPIDController();
    rev::CANPIDController m_shooterMotorRightController = m_shooterMotorRight.GetPIDController();
    rev::CANPIDController m_rotationMotorController = m_rotationMotor.GetPIDController();

    intakeshooterStates currentIntakeshooterState = intakeshooterStates::IDLE;
};
