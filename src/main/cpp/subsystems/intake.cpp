#pragma once
#include "subsystems/intake.h"

intake::intake()
: m_intakeMotorLeft(intakeConstants::kIntakeMotorLeft),
m_intakeMotorRight(intakeConstants::kIntakeMotorRight){

    m_intakeMotorRight.SetControl(ctre::phoenix6::controls::Follower(intakeConstants::kIntakeMotorLeft, true));

    ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    ctre::phoenix6::configs::Slot0Configs slotZeroConfigs{};


    motorOutputConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    motorOutputConfigs.WithPeakForwardDutyCycle(1.0);
    motorOutputConfigs.WithPeakReverseDutyCycle(-1.0);
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

}

void intake::IntakeToggle(){
    if (intakeOn == true){
        m_intakeMotorLeft.Set(0.2);
    }
    else if (intakeOn == false){
        m_intakeMotorLeft.Set(0.0);
    }
    intakeOn = !intakeOn;
}

void intake::IntakeOn() {

}

void intake::IntakeStop() {

}

void intake::IntakeIdle() {
    
}