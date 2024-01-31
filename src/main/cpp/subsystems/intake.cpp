#pragma once
#include "subsystems/intake.h"

intake::intake()
: m_rollerMotor1(intakeConstants::kMotorRollerLeft, rev::CANSparkMax::MotorType::kBrushless),
m_rollerMotor2(intakeConstants::kMotorRollerRight, rev::CANSparkMax::MotorType::kBrushless){

    m_rollerMotor1.RestoreFactoryDefaults();
    m_rollerMotor2.RestoreFactoryDefaults();

    m_rollerMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rollerMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_rollerMotor1.SetSmartCurrentLimit(40.0);
    m_rollerMotor2.SetSmartCurrentLimit(40.0);

    m_rollerMotor2.Follow(m_rollerMotor1, true);

    m_rollerMotor1Controller.SetP(0);
    m_rollerMotor1Controller.SetI(0);
    m_rollerMotor1Controller.SetD(0);
    m_rollerMotor1Controller.SetFF(0);
    m_rollerMotor1Controller.SetOutputRange(-1.0F, 1.0F);
}

void intake::IntakeToggle(){
    if (intakeOn == true){
        m_rollerMotor1.Set(0.2);
    }
    else if (intakeOn == false){
        m_rollerMotor1.Set(0.0);
    }
    intakeOn = !intakeOn;
}