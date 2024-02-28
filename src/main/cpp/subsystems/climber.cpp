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

    m_climberMotorLeft.SetSmartCurrentLimit(80.0);
    m_climberMotorRight.SetSmartCurrentLimit(80.0);

    m_climberMotorLeft.SetInverted(true);

    // PID for Climber Motor Left and Right
    m_climberMotorLeftController.SetP(0.005);
    m_climberMotorLeftController.SetI(0);
    m_climberMotorLeftController.SetD(0);
    m_climberMotorLeftController.SetFF(0);
    m_climberMotorLeftController.SetOutputRange(-1.0F,1.0F);

    m_climberMotorLeftEncoder.SetPositionConversionFactor((1.0 / kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorLeftEncoder.SetVelocityConversionFactor(((1.0 / kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);

    m_climberMotorRightController.SetP(0.005);
    m_climberMotorRightController.SetI(0);
    m_climberMotorRightController.SetD(0);
    m_climberMotorRightController.SetFF(0);
    m_climberMotorRightController.SetOutputRange(-1.0F,1.0F);

    m_climberMotorRightEncoder.SetPositionConversionFactor((1.0 / kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorRightEncoder.SetVelocityConversionFactor(((1.0 / kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);

    //Left and Right Climber Solenoids
    m_climberSolenoidLeft.RestoreFactoryDefaults();
    m_climberSolenoidRight.RestoreFactoryDefaults();

    m_climberSolenoidLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberSolenoidRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberSolenoidLeft.SetSmartCurrentLimit(40.0);
    m_climberSolenoidRight.SetSmartCurrentLimit(40.0);

    // Set Left Motors and Solenoids to follow Right
    // m_climberMotorRight.Follow(m_climberMotorLeft, true);
    m_climberSolenoidRight.Follow(m_climberSolenoidLeft, false);
}

/* // Climber state machene (toggle and explicit set)
void climber::TelescopeToggle () { // Note that turnery would need to be expanded with addition of any additional states (get S to do it)
    currentTelescopeState = (currentTelescopeState == telescopeStates::UNEXTENDED) ? telescopeStates::EXTENDED : telescopeStates::UNEXTENDED;
}
void climber::TelescopeToggle (telescopeStates state) {
    currentTelescopeState = state;
}

// Set height of climber
void climber::SetHeight(double height){
	m_climberMotorLeftController.SetReference(height, rev::CANSparkMax::ControlType::kPosition);
} */

void climber::disengageSolenoids() {
    m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(0));
}

void climber::zeroClimber() {
    // currentZeroState = zeroingStates::MANUALZERO;

    m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(12));
}

void climber::climberRetract() {
    m_climberMotorLeft.Set(0.0);
    m_climberMotorRight.Set(0.0);

    //m_climberMotorLeftEncoder.SetPosition(0.0);
}

void climber::climberExtend() {
    m_climberMotorLeft.Set(0.2);
    m_climberMotorRight.Set(0.2);

    //m_climberMotorLeftController.SetReference(10.0, rev::CANSparkMax::ControlType::kPosition);
}

void climber::motorRetract() {
    m_climberMotorLeft.Set(-0.2);
    m_climberMotorRight.Set(-0.2);

    //m_climberMotorLeftController.SetReference(0.0, rev::CANSparkMax::ControlType::kPosition);
}

void climber::Periodic(){
    frc::SmartDashboard::PutNumber("Tele Pos", m_climberMotorLeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climb Out Curr", m_climberMotorLeft.GetOutputCurrent());
    frc::SmartDashboard::PutString("Zero State", zeroState);

    switch (currentZeroState) {
        case zeroingStates::MANUALZERO:
            currentZeroState = zeroingStates::NOTZEROED;
        
            zeroState = "MANUALZERO";
            break;
        case zeroingStates::INIT:
            m_climberMotorLeft.Set(-0.1);
            if (m_climberMotorLeft.GetOutputCurrent() > 20) {
                m_climberMotorLeftEncoder.SetPosition(0);
                currentZeroState = zeroingStates::ZEROED;
            }

            zeroState = "INIT";
            break;
        case zeroingStates::NOTZEROED:
            currentZeroState = zeroingStates::INIT;

            zeroState = "NOTZEROED";
            break;
        case zeroingStates::ZEROED:
            m_climberMotorLeft.Set(0.0);
            currentZeroState = zeroingStates::IDLE;

            zeroState = "ZEROED";
            break;
        
        default:
        case zeroingStates::IDLE:

            zeroState = "IDLE";
            break;
    }

    switch (currentExtendState) {
        case extendingStates::EXTEND:
            m_climberMotorLeftController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion);
            m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(12));

            currentExtendState = extendingStates::WAITING;

            break;
        case extendingStates::WAITING:
            if (m_climberMotorLeftEncoder.GetPosition() > 0.0) {
                currentExtendState = extendingStates::CLOSESOLENOIDS;
            }

            break;
        case extendingStates::CLOSESOLENOIDS:
            m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(0)); //12

            currentExtendState = extendingStates::INIT;
            break;
        
        default:
        case extendingStates::INIT:

            break;
    }

    /* switch (currentTelescopeState) {
    case telescopeStates::UNEXTENDED:
        m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(-12));
        SetHeight(0.0);
        break;
    case telescopeStates::EXTENDED:
        m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(12));
        SetHeight(2.0);
    default:
        break;
    } */


}