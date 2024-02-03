#pragma once
#include "subsystems/climber.h"

climber::climber()
: m_climberMotor1(1, rev::CANSparkMax::MotorType::kBrushless),
m_climberMotor2(2, rev::CANSparkMax::MotorType::kBrushless){
    m_climberMotor1.RestoreFactoryDefaults();
    m_climberMotor2.RestoreFactoryDefaults();

    m_climberMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberMotor1.SetSmartCurrentLimit(40.0);
    m_climberMotor2.SetSmartCurrentLimit(40.0);

    m_climberMotor2.Follow(m_climberMotor1, true);

    m_climberMotor1Controller.SetP(0);
    m_climberMotor1Controller.SetI(0);
    m_climberMotor1Controller.SetD(0);
    m_climberMotor1Controller.SetFF(0);
    m_climberMotor1Controller.SetOutputRange(-1.0F,1.0F);
    
}

// void climber::ClimberExtend() {
//     m_climberMotor1.Set(0.3);
// }

// void climber::ClimberRetract() {
//     m_climberMotor1.Set(-0.3);
// }


// Set height of climber
void climber::SetHeight(double height){
	m_climberMotor1.Set(height);
}