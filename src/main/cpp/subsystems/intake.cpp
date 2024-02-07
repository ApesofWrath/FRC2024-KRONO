#pragma once
#include "subsystems/intake.h"

intake::intake()
: m_intakeMotorLeft(intakeConstants::kIntakeMotorLeft, rev::CANSparkMax::MotorType::kBrushless),
m_intakeMotorRight(intakeConstants::kIntakeMotorRight, rev::CANSparkMax::MotorType::kBrushless){

    m_intakeMotorLeft.RestoreFactoryDefaults();
    m_intakeMotorRight.RestoreFactoryDefaults();

    m_intakeMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_intakeMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_intakeMotorLeft.SetSmartCurrentLimit(40.0);
    m_intakeMotorRight.SetSmartCurrentLimit(40.0);

    m_intakeMotorRight.Follow(m_intakeMotorLeft, true);

    m_intakeMotorLeftController.SetP(0);
    m_intakeMotorLeftController.SetI(0);
    m_intakeMotorLeftController.SetD(0);
    m_intakeMotorLeftController.SetFF(0);
    m_intakeMotorLeftController.SetOutputRange(-1.0F, 1.0F);
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

void intake::IntakeToggle() {
    
}