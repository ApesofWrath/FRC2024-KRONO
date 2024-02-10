#pragma once
#include "subsystems/climber.h"

climber::climber()
: m_climberMotorLeft(kMotorClimberLeft, rev::CANSparkMax::MotorType::kBrushless),
m_climberMotorRight(kMotorClimberRight, rev::CANSparkMax::MotorType::kBrushless){
    m_climberMotorLeft.RestoreFactoryDefaults();
    m_climberMotorRight.RestoreFactoryDefaults();

    m_climberMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberMotorLeft.SetSmartCurrentLimit(40.0);
    m_climberMotorRight.SetSmartCurrentLimit(40.0);

    m_climberMotorLeftController.SetP(0);
    m_climberMotorLeftController.SetI(0);
    m_climberMotorLeftController.SetD(0);
    m_climberMotorLeftController.SetFF(0);
    m_climberMotorLeftController.SetOutputRange(-1.0F,1.0F);

    m_climberMotorLeftEncoder.SetPositionConversionFactor((1 / kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorLeftEncoder.SetVelocityConversionFactor(((1 / kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60);

    m_climberMotorRightController.SetP(0);
    m_climberMotorRightController.SetI(0);
    m_climberMotorRightController.SetD(0);
    m_climberMotorRightController.SetFF(0);
    m_climberMotorRightController.SetOutputRange(-1.0F,1.0F);

    m_climberMotorRightEncoder.SetPositionConversionFactor((1 / kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorRightEncoder.SetVelocityConversionFactor(((1 / kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60);

    m_climberMotorRight.Follow(m_climberMotorLeft, true);
}

// Climber state machene (toggle and explicit set)
void climber::TelescopeToggle () { // Note that turnery would need to be expanded with addition of any additional states (get S to do it)
    currentTelescopeState = (currentTelescopeState == telescopeStates::UNEXTENDED) ? telescopeStates::EXTENDED : telescopeStates::UNEXTENDED;
}
void climber::TelescopeToggle (telescopeStates state) {
    currentTelescopeState = state;
}

// Set height of climber
void climber::SetHeight(double height){
	m_climberMotorLeftController.SetReference(height, rev::CANSparkMax::ControlType::kPosition);
}
void climber::Periodic(){
    switch (currentTelescopeState)
    {
    case telescopeStates::UNEXTENDED:
        SetHeight(0.0);
        break;
    case telescopeStates::EXTENDED:
        SetHeight(2.0);
    default:
        break;
    }
}