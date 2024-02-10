#pragma once
#include "subsystems/intake.h"

intake::intake()
: m_intakeMotorLeft(intakeConstants::kIntakeMotorLeft),
m_intakeMotorRight(intakeConstants::kIntakeMotorRight){

    
    m_intakeMotorRight.SetControl(ctre::phoenix6::controls::Follower(intakeConstants::kIntakeMotorLeft, true));

    motorOutputConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    currentLimitsConfigs.WithStatorCurrentLimitEnable(true);
    currentLimitsConfigs.WithStatorCurrentLimit(40.0);
    slotZeroConfigs.WithKP(0.0);
    slotZeroConfigs.WithKI(0.0);
    slotZeroConfigs.WithKD(0.0);
    m_intakeMotorLeft.GetConfigurator().Apply(motorOutputConfigs);
    m_intakeMotorLeft.GetConfigurator().Apply(currentLimitsConfigs);
    m_intakeMotorLeft.GetConfigurator().Apply(slotZeroConfigs);
    m_intakeMotorRight.GetConfigurator().Apply(motorOutputConfigs);
    m_intakeMotorRight.GetConfigurator().Apply(currentLimitsConfigs);
    m_intakeMotorRight.GetConfigurator().Apply(slotZeroConfigs);

    m_velocity.WithSlot(0);
    m_velocity.WithEnableFOC(false);

}

void intake::IntakeToggle() {
    if (currentIntakeSpeedState == IntakeSpeedStates::OFF){
        currentIntakeSpeedState == IntakeSpeedStates::ON;
    }
    else if (currentIntakeSpeedState == IntakeSpeedStates::ON){
        currentIntakeSpeedState == IntakeSpeedStates::OFF;
    }
}

void intake::Periodic() {
    switch (currentIntakeSpeedState) {
        case IntakeSpeedStates::OFF:
            m_intakeMotorLeft.SetControl(m_velocity.WithVelocity(0_tps));
            break;
        case IntakeSpeedStates::IDLE:
            m_intakeMotorLeft.SetControl(m_velocity.WithVelocity(2.5_tps));
            break;
        case IntakeSpeedStates::ON:
            m_intakeMotorLeft.SetControl(m_velocity.WithVelocity(10.0_tps));
            break;
    }
}