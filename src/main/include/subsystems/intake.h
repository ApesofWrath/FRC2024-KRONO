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


enum class IntakeSpeedStates {
    ON,
    OFF,
    IDLE
};


class intake : public frc2::SubsystemBase {
    public:
    intake();
    void IntakeToggle();

    void Periodic() override;
    private:
    //rev::CANSparkMax m_intakeMotorLeft;
    //rev::CANSparkMax m_intakeMotorRight;
    ctre::phoenix6::hardware::TalonFX m_intakeMotorLeft;
    ctre::phoenix6::hardware::TalonFX m_intakeMotorRight;

    ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    ctre::phoenix6::configs::Slot0Configs slotZeroConfigs{};

    ctre::phoenix6::controls::VelocityDutyCycle m_velocity{0_tps};
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();

    //rev::SparkMaxPIDController m_intakeMotorLeftController = m_intakeMotorLeft.GetPIDController();
    //rev::SparkMaxPIDController m_intakeMotorRightController = m_intakeMotorRight.GetPIDController();
    IntakeSpeedStates currentIntakeSpeedState = IntakeSpeedStates::OFF;
    bool intakeOn = true;
};
