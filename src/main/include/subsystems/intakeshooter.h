#pragma once


#include <numbers>
#include <string>
#include <iostream>
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/CANifier.h>
#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/XboxController.h>
#include <frc2/command/button/CommandXboxController.h>


enum class intakeshooterStates { // proceed cyclically down list, each comment describes state & conditions for entering
    IDLE, // default state, wheels (except feeder) spinning slowly
    INTAKING, // enter on intake button, intake angles down & spins wheels
    BACKOFF, // feed note backwards out of mechanism to prevent excess grinding & early shots
    NOTEFORWARD,
    HOLDING, // enter on note at correct position (sensor), ensure note position correctness
    SPINUP, // enter on rev button press, firing wheels go to max speed & angle correctly (read shootTarget)
    SPINUPPIGEON,
    AMPBACK,
    AIMAMP,
    SCOREAMP,
    RAPIDFIRE, // do not bind to a button - for auto use only(goes to rapidpostfire state)
    FIRE, // enter fire button, feed note to shooter wheels, go to idle when note gone
    POSTFIRE, // after the note is fired, check if the note is gone before resuming idle
    RAPIDPOSTFIRE, // returns to intaking position instead of idle for quicker shots and less intake movement
    ZEROING // after we fire, go back to neutral and then resume idle
};

class intakeshooter : public frc2::SubsystemBase {
    public:
    intakeshooter(frc2::CommandXboxController* controllerMain, frc2::CommandXboxController* controllerOperator, ctre::phoenix::CANifier& beambreakCanifier);
    void intakeActivate();
    void intakeRetract();
    void spinup();
    void scoreAmp();
    void spinup(float angle);
    void fire();
    void rapidFire();
    intakeshooterStates getState();
    bool shooterAtSpeed();
	frc2::CommandPtr zeroOTF();

    bool allowSpinup = true;

    void Periodic() override;
    private:


    frc2::CommandXboxController* m_controllerMain;
    frc2::CommandXboxController* m_controllerOperator;

    ctre::phoenix6::hardware::TalonFX m_intakeMotorLeft;
    ctre::phoenix6::hardware::TalonFX m_intakeMotorRight;

    ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    ctre::phoenix6::configs::Slot0Configs slotZeroConfigs{};

    ctre::phoenix6::controls::VelocityDutyCycle m_velocityIntake{0_tps};

    ctre::phoenix::CANifier& m_BeambreakCanifier;

    ctre::phoenix6::hardware::Pigeon2 m_Pigeon;

    rev::CANSparkMax m_shooterMotorLeft;
    rev::CANSparkMax m_shooterMotorRight;
    rev::CANSparkMax m_rotationMotor;
    rev::SparkMaxAlternateEncoder m_rotationEncoder = m_rotationMotor.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::Type::kQuadrature, 8192);

    rev::SparkRelativeEncoder m_shooterLeftEncoder = m_shooterMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder m_shooterRightEncoder = m_shooterMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

    rev::SparkPIDController m_shooterMotorLeftController = m_shooterMotorLeft.GetPIDController();
    rev::SparkPIDController m_shooterMotorRightController = m_shooterMotorRight.GetPIDController();
    rev::SparkPIDController m_rotationMotorController = m_rotationMotor.GetPIDController();

    intakeshooterStates currentIntakeshooterState = intakeshooterStates::IDLE;
    int counter;

    std::string intakeState = ""; // display the intake state as a string for smartDash, no elegant way to do this so dont bother

    int shooterClearCount = 0;
    double shootAngle; // set the angle at which we are shooting based off of the limelight
    double gravityFF = 0.0; // calculate to conteract the force of gravity when setting the angle
    int ampBackCount = 0;

    double rollingSamples[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int rollingSample = 0;
    double rollSampSum = 0.0;
    double rollSampAvg = 0.0;
};
