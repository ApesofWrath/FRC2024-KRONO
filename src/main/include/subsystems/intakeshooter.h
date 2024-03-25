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
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/XboxController.h>
#include <frc2/command/button/CommandXboxController.h>

enum class intakeshooterStates { // Enter condition [id BUTTON] -> action [-> exit condition -> next state]
    IDLE, // DEFAULT, intakeRetract [op A], transition from IDLE -> apply current limits, go to angle zero speed zero
    INTAKING, // intakeActivate [op LB], transition from RAPIDPOSTFIRE -> intake angles down & spins wheels -> canifier LIMR tripped -> BACKOFF
    BACKOFF, // transition from INTAKING -> retract, feed note backwards out of mechanism (prevent excess grinding & early shots) -> canifier LIMR tripped -> NOTEFORWARD
    NOTEFORWARD, // transition from BACKOFF -> move note forward, rumble controllers -> canifier LIMR tripped -> HOLDING
    HOLDING, // transition from NOTEFORWARD -> stop moving the note, allowSpinup
    SPINUP, // spinup [op B X] -> angle correctly for speaker and get shooter wheels up to speed -> correct position -> SPINUPPIGEON
    SPINUPPIGEON, // transition from SPINUP -> correct angle using pigeon
    AIMAMP, // scoreAmp [op Y], transition from AIMAMP -> angle correctly for amp -> amp angle is correct -> SCOREAMP 
    SCOREAMP, // transition from AIMAMP -> wait 50 loops, apply current limit, spin left motor
    FIRE, // fire [op RB] -> ensure correct angle, spin up left wheel, iterate shooterClearCount -> canifyer LIMF tripped -> POSTFIRE
    POSTFIRE, // after rapidFireCommand [auton], after fire, transition from FIRE -> reset shooterClearCount -> canifyer LIMF tripped -> IDLE
    RAPIDFIRE, // rapidFire [auton] -> set the correct angle, spin up left wheel -> canifier LIMF tripped -> RAPIDPOSTFIRE
    RAPIDPOSTFIRE, // transition from RAPIDFIRE -> -> canifier LIMF tripped -> INTAKING
};

class intakeshooter : public frc2::SubsystemBase {
    public:
    intakeshooter(frc2::CommandXboxController* controllerMain, frc2::CommandXboxController* controllerOperator);
    void intakeActivate();
    void intakeRetract();
    void spinup();
    void scoreAmp();
    void spinup(float angle);
    void fire();
    void rapidFire();
    intakeshooterStates getState();
    bool shooterAtSpeed();

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

    ctre::phoenix::CANifier m_BeambreakCanifier;

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

    std::string intakeState = ""; // display the intake state as a string for smartDash, no elegant way to do this so dont bother

    int shooterClearCount = 0;
    double shootAngle; // set the angle at which we are shooting based off of the limelight
    double gravityFF = 0.0; // calculate to conteract the force of gravity when setting the angle
    int ampBackCount = 0;
    int ampWaitCounter = 0;
};
