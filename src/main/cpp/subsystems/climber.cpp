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
}

void climber::ClimberExtend() {
    m_climberMotor1.Set(0.3);
}

void climber::ClimberRetract() {
    m_climberMotor1.Set(-0.3);
}