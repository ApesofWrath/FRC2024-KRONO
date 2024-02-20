#pragma once
#include "subsystems/climber.h"

climber::climber()
: m_climberMotorLeft(kMotorClimberLeft, rev::CANSparkMax::MotorType::kBrushless),
m_climberMotorRight(kMotorClimberRight, rev::CANSparkMax::MotorType::kBrushless), 
m_climberSolenoidLeft(kSolenoidClimberLeft, rev::CANSparkMax::MotorType::kBrushed), 
m_climberSolenoidRight(kSolenoidClimberRight, rev::CANSparkMax::MotorType::kBrushed)
{
    // Left and Right Climber Motors
    m_climberMotorLeft.RestoreFactoryDefaults();
    m_climberMotorRight.RestoreFactoryDefaults();

    m_climberMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberMotorLeft.SetSmartCurrentLimit(40.0);
    m_climberMotorRight.SetSmartCurrentLimit(40.0);

    // PID for Climber Motor Left
    m_climberMotorLeftController.SetP(0);
    m_climberMotorLeftController.SetI(0);
    m_climberMotorLeftController.SetD(0);
    m_climberMotorLeftController.SetFF(0);
    m_climberMotorLeftController.SetOutputRange(-1.0F,1.0F);

    m_climberMotorLeftEncoder.SetPositionConversionFactor((1 / kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorLeftEncoder.SetVelocityConversionFactor(((1 / kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60);

    /* m_climberMotorRightController.SetP(0);
    m_climberMotorRightController.SetI(0);
    m_climberMotorRightController.SetD(0);
    m_climberMotorRightController.SetFF(0);
    m_climberMotorRightController.SetOutputRange(-1.0F,1.0F);

    m_climberMotorRightEncoder.SetPositionConversionFactor((1 / kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorRightEncoder.SetVelocityConversionFactor(((1 / kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60); */

    //Left and Right Climber Solenoids
    m_climberSolenoidLeft.RestoreFactoryDefaults();
    m_climberSolenoidRight.RestoreFactoryDefaults();

    m_climberSolenoidLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberSolenoidRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberSolenoidLeft.SetSmartCurrentLimit(40.0);
    m_climberSolenoidRight.SetSmartCurrentLimit(40.0);

    // Set Left Motors and Solenoids to follow Right
    m_climberMotorRight.Follow(m_climberMotorLeft, true);
    m_climberSolenoidRight.Follow(m_climberSolenoidLeft, false);
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
    switch (currentTelescopeState) {
    case telescopeStates::UNEXTENDED:
        m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(-12));
        SetHeight(0.0);
        break;
    case telescopeStates::EXTENDED:
        m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(12));
        SetHeight(2.0);
    default:
        break;
    }
}