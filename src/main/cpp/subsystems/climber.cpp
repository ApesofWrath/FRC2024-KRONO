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

    m_climberMotorLeft.SetInverted(true);

    // PID for Climber Motor Left and Right
    m_climberMotorLeftController.SetP(0.65); //0.15
    m_climberMotorLeftController.SetI(0.0004);
    m_climberMotorLeftController.SetD(0);
    m_climberMotorLeftController.SetFF(0);
    m_climberMotorLeftController.SetOutputRange(-1.0F,1.0F);
    m_climberMotorLeftController.SetIZone(0.25);

    m_climberMotorLeftEncoder.SetPositionConversionFactor((kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorLeftEncoder.SetVelocityConversionFactor(((kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);

    m_climberMotorRightController.SetP(1.05); //0.55
    m_climberMotorRightController.SetI(0.0004);
    m_climberMotorRightController.SetD(0);
    m_climberMotorRightController.SetFF(0);
    m_climberMotorRightController.SetOutputRange(-1.0F,1.0F);
    m_climberMotorRightController.SetIZone(0.25);

    m_climberMotorRightEncoder.SetPositionConversionFactor((kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorRightEncoder.SetVelocityConversionFactor(((kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);

    //Left and Right Climber Solenoids
    m_climberSolenoidLeft.RestoreFactoryDefaults();
    m_climberSolenoidRight.RestoreFactoryDefaults();

    m_climberSolenoidLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberSolenoidRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberSolenoidLeft.SetSmartCurrentLimit(40.0);
    m_climberSolenoidRight.SetSmartCurrentLimit(40.0);

    // Set Left Solenoid to follow Right
    m_climberSolenoidRight.Follow(m_climberSolenoidLeft, false);

    // Set Climber Positions for Encoders
    m_climberMotorLeftEncoder.SetPosition(0.2);
    m_climberMotorRightEncoder.SetPosition(0.2);

    // Burn all settings
    m_climberMotorLeft.BurnFlash();
    m_climberMotorRight.BurnFlash();
    m_climberSolenoidLeft.BurnFlash();
    m_climberSolenoidRight.BurnFlash();
}

void climber::climberRetract() {
    currentExtendState = extendingStates::RETRACT;
}

void climber::climberExtend() {
    currentExtendState = extendingStates::SOLEXTEND;
}

void climber::leftClimbToggle() {
    m_climberMotorLeft.Set(lToggle ? 0.0 : -0.2);
    if (lToggle) { m_climberMotorLeftEncoder.SetPosition(0); } // TODO: what's the point of setting the encoder here?
    lToggle = !lToggle;
}

void climber::rightClimbToggle() {
    m_climberMotorLeft.Set(rToggle ? 0.0 : -0.2);
    if (rToggle) { m_climberMotorRightEncoder.SetPosition(0); }
    rToggle = !rToggle;
}

void climber::Periodic(){
    frc::SmartDashboard::PutNumber("Tele Pos Left", m_climberMotorLeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Tele Pos Right", m_climberMotorRightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climb Out Curr L", m_climberMotorLeft.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Climb Out Curr R", m_climberMotorRight.GetOutputCurrent());
    frc::SmartDashboard::PutString("Climb State", climbState);

    switch (currentExtendState) {
        case extendingStates::SOLEXTEND:
            m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(12));

            solCount++;

            if (solCount > 4) {
                m_climberMotorLeft.Set(-0.05);
                m_climberMotorRight.Set(-0.05);
                currentExtendState = extendingStates::EXTEND;
                solCount = 0;
            }

            climbState = "SOLEXTEND";
            break;
        case extendingStates::EXTEND:
            m_climberMotorLeftController.SetReference(19, rev::CANSparkMax::ControlType::kPosition);
            m_climberMotorRightController.SetReference(20, rev::CANSparkMax::ControlType::kPosition);

            climbState = "EXTEND";
            currentExtendState = extendingStates::INIT;
            break;
        case extendingStates::RETRACT:
            m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(0));

            m_climberMotorLeftController.SetReference(0.5, rev::CANSparkMax::ControlType::kPosition);
            m_climberMotorRightController.SetReference(0.5, rev::CANSparkMax::ControlType::kPosition);

            if (m_climberMotorLeftEncoder.GetPosition() < 0.8 && m_climberMotorRightEncoder.GetPosition() < 0.8) {
                currentExtendState = extendingStates::POSTRETRACT;
            }
            
            climbState = "RETRACT";
            break;
        case extendingStates::POSTRETRACT:
            m_climberMotorLeft.Set(0);
            m_climberMotorRight.Set(0);

            climbState = "POSTRETRACT";
            currentExtendState = extendingStates::INIT;
            break;
        case extendingStates::INIT:

            climbState = "INIT";
            break;
    }
}