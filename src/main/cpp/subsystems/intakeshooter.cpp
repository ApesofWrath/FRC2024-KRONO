#pragma once
#include "subsystems/intakeshooter.h"
using namespace shooterConstants;
using namespace intakeConstants;

intakeshooter::intakeshooter()
: m_intakeMotorLeft(kIntakeMotorLeft),
m_intakeMotorRight(kIntakeMotorRight),
m_shooterMotorLeft(kMotorShooterLeft, rev::CANSparkMax::MotorType::kBrushless),
m_shooterMotorRight(kMotorShooterRight, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(kIntakeRotationMotor, rev::CANSparkMax::MotorType::kBrushless){

    //Krakens
    m_intakeMotorRight.SetControl(ctre::phoenix6::controls::Follower(kIntakeMotorLeft, true));

    motorOutputConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    currentLimitsConfigs.WithStatorCurrentLimitEnable(true);
    currentLimitsConfigs.WithStatorCurrentLimit(40.0);
    slotZeroConfigs.WithKP(0.0);

    for (ctre::phoenix6::hardware::TalonFX motor : new ctre::phoenix6::hardware::TalonFX[]{m_intakeMotorLeft, m_intakeMotorRight}) {
        for (ctre::phoenix6::configs::MotorOutputConfigs config : new ctre::phoenix6::configs::MotorOutputConfigs[]{motorOutputConfigs, currentLimitsConfigs, slotZeroConfigs}) {
            motor.GetConfigurator().Apply(config);
        }
    }

    m_velocity.WithSlot(0);
    m_velocity.WithEnableFOC(false);

    //Neos
    for (rev::CANSparkMax motor : rev::CANSparkMax[]{m_shooterMotorLeft, m_shooterMotorRight, m_rotationMotor}){
        motor.RestoreFactoryDefaults();
        motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        motor.SetSmartCurrentLimit(40.0);
    }

    m_shooterMotorRight.Follow(m_shooterMotorLeft, true);

    for (rev::CANPIDController controller : rev::CANPIDController[]{m_shooterMotorLeftController, m_shooterMotorRightController, m_rotationMotorController}){
        controller.SetP(0);
        controller.SetI(0);
        controller.SetD(0);
        controller.SetFF(0);
        controller.SetOutputRange(-1.0F, 1.0F);
    }
}

void intakeshooter::intakeActivate() {
    currentIntakeshooterState = intakeshooterStates::IDLE;
}
void intakeshooter::spinup() {
    currentIntakeshooterState = intakeshooterStates::SPINUP;
}
void intakeshooter::fireSPEAKER() {
    currentShootTarget = shootTarget::SPEAKER;
    currentIntakeshooterState = intakeshooterStates::FIRE;
}
void intakeshooter::fireAMP() {
    currentShootTarget = shootTarget::AMP;
    currentIntakeshooterState = intakeshooterStates::FIRE;
}

void intakeshooter::Periodic() {
    // intakeshooter state machene
    switch (currentIntakeshooterState) {
        case intakeshooterStates::IDLE:
            break;
        case intakeshooterStates::INTAKEING:
            break;
        case intakeshooterStates::HOLDING:
            break;
        case intakeshooterStates::SPINUP:
            break;
        case intakeshooterStates::FIRE:
            break;
    }
}