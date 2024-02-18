#pragma once

#include <numbers>
#include <string>
#include <iostream>
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/CANifier.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

enum class intakeshooterStates { // proceed cyclically down list, each comment describes state & conditions for entering
    IDLE, // default state, wheels (except feeder) spinning slowly
    INTAKING, // enter on intake button, intake angles down & spins wheels
    HOLDING, // enter on note at correct position (sensor), ensure note position correctness
    SPINUP, // enter on rev button press, firing wheels go to max speed & angle correctly (read shootTarget)
    FIRE // enter fire button, feed note to shooter wheels, go to idle when note gone
};

enum class shootTarget { // set this based off of which `fire` button is pressed
    AMP, // angle the shooter to be able to go for the amp
    SPEAKER // angle the shooter to be able to go for the shooter
};

class intakeshooter : public frc2::SubsystemBase {
    public:
    intakeshooter();
    void intakeActivate();
    void spinup();
    void fireSPEAKER();
    void fireAMP();

    void Periodic() override;
    private:
    ctre::phoenix6::hardware::TalonFX m_intakeMotorLeft;
    ctre::phoenix6::hardware::TalonFX m_intakeMotorRight;

    ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    ctre::phoenix6::configs::Slot0Configs slotZeroConfigs{};

    ctre::phoenix6::controls::VelocityDutyCycle m_velocityIntake{0_tps};

    ctre::phoenix::CANifier m_BeambreakCanifier;

    rev::CANSparkMax m_shooterMotorLeft;
    rev::CANSparkMax m_shooterMotorRight;
    rev::CANSparkMax m_rotationMotor;
    rev::SparkMaxAlternateEncoder m_rotationEncoder = m_rotationMotor.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::AlternateEncoderType::kQuadrature, 8192);

    rev::SparkPIDController m_shooterMotorLeftController = m_shooterMotorLeft.GetPIDController();
    rev::SparkPIDController m_shooterMotorRightController = m_shooterMotorRight.GetPIDController();
    rev::SparkPIDController m_rotationMotorController = m_rotationMotor.GetPIDController();

    intakeshooterStates currentIntakeshooterState = intakeshooterStates::IDLE;
    shootTarget currentShootTarget = shootTarget::AMP;

    std::string intakeState = "";
};
