#pragma once
#include "subsystems/climber.h"

climber::climber()
: m_climberMotorLeft(climberConstants::kMotorClimberLeft, rev::CANSparkMax::MotorType::kBrushless),
m_climberMotorRight(climberConstants::kMotorClimberRight, rev::CANSparkMax::MotorType::kBrushless){
    m_climberMotorLeft.RestoreFactoryDefaults();
    m_climberMotorRight.RestoreFactoryDefaults();

    m_climberMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberMotorLeft.SetSmartCurrentLimit(40.0);
    m_climberMotorRight.SetSmartCurrentLimit(40.0);

    m_climberMotorRight.Follow(m_climberMotorLeft, true);

    m_climberMotorLeftController.SetP(0);
    m_climberMotorLeftController.SetI(0);
    m_climberMotorLeftController.SetD(0);
    m_climberMotorLeftController.SetFF(0);
    m_climberMotorLeftController.SetOutputRange(-1.0F,1.0F);
}

// void climber::ClimberExtend() {
//     m_climberMotor1.Set(0.3);
// }

// void climber::ClimberRetract() {
//     m_climberMotor1.Set(-0.3);
// }


// Set height of climber
void climber::SetHeight(double height){
	m_climberMotorLeft.Set(height);
}