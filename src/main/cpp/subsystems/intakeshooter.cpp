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

    m_intakeMotorLeft.GetConfigurator().Apply(motorOutputConfigs);
    m_intakeMotorLeft.GetConfigurator().Apply(currentLimitsConfigs);
    m_intakeMotorLeft.GetConfigurator().Apply(slotZeroConfigs);

    m_intakeMotorRight.GetConfigurator().Apply(motorOutputConfigs);
    m_intakeMotorRight.GetConfigurator().Apply(currentLimitsConfigs);
    m_intakeMotorRight.GetConfigurator().Apply(slotZeroConfigs);

    m_velocity.WithSlot(0);
    m_velocity.WithEnableFOC(false);

    //Neos
    m_shooterMotorLeft.RestoreFactoryDefaults();
    m_shooterMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterMotorLeft.SetSmartCurrentLimit(40.0);

    m_shooterMotorRight.RestoreFactoryDefaults();
    m_shooterMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterMotorRight.SetSmartCurrentLimit(40.0);
    
    m_rotationMotor.RestoreFactoryDefaults();
    m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rotationMotor.SetSmartCurrentLimit(40.0);

    m_shooterMotorRight.Follow(m_shooterMotorLeft, true);

    m_shooterMotorLeftController.SetP(0);
    m_shooterMotorLeftController.SetI(0);
    m_shooterMotorLeftController.SetD(0);
    m_shooterMotorLeftController.SetFF(0);
    m_shooterMotorLeftController.SetOutputRange(-1.0F, 1.0F);

    m_shooterMotorRightController.SetP(0);
    m_shooterMotorRightController.SetI(0);
    m_shooterMotorRightController.SetD(0);
    m_shooterMotorRightController.SetFF(0);
    m_shooterMotorRightController.SetOutputRange(-1.0F, 1.0F);

    m_rotationMotorController.SetP(0);
    m_rotationMotorController.SetI(0);
    m_rotationMotorController.SetD(0);
    m_rotationMotorController.SetFF(0);
    m_rotationMotorController.SetOutputRange(-1.0F, 1.0F);
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