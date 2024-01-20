#pragma once
#include "subsystems/intake.h"

intake::intake()
: m_motor1(20, rev::CANSparkMax::MotorType::kBrushless),
m_motor2(30, rev::CANSparkMax::MotorType::kBrushless){

    m_motor1.RestoreFactoryDefaults();
    m_motor2.RestoreFactoryDefaults();

    m_motor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_motor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_motor1.SetSmartCurrentLimit(40.0);
    m_motor2.SetSmartCurrentLimit(40.0);
    }

    void intake::IntakeToggle(){
        if (intakeOn == false){
            m_motor1.Set(-0.2);
            m_motor2.Set(-0.2);
        }
        else if (intakeOn == true){
            m_motor1.Set(0.0);
            m_motor2.Set(0.0);
        }
        intakeOn = !intakeOn;
    }
