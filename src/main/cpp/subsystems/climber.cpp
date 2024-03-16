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
    std::vector<std::function<rev::REVLibError()>> climberMotorLeftConfigs = {
        [this]() {return m_climberMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);},
        [this]() {return m_climberMotorLeft.SetSmartCurrentLimit(40.0);},
        [this]() {return m_sparkUtil.setInvert(&m_climberMotorLeft, true);},
        [this]() {return m_climberMotorLeftController.SetP(0.65);},
        [this]() {return m_climberMotorLeftController.SetI(0.0004);},
        [this]() {return m_climberMotorLeftController.SetD(0.0);},
        [this]() {return m_climberMotorLeftController.SetFF(0.0);},
        [this]() {return m_climberMotorLeftController.SetOutputRange(-1.0F,1.0F);},
        [this]() {return m_climberMotorLeftController.SetIZone(0.25);},
        [this]() {return m_climberMotorLeftEncoder.SetPositionConversionFactor((kRotationsToInchTelescoping) * (kTelescopingRatio));},
        [this]() {return m_climberMotorLeftEncoder.SetVelocityConversionFactor(((kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);}
    };

    m_sparkUtil.configure(&m_climberMotorLeft, climberMotorLeftConfigs);
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

void climber::climberRetract() {
    currentExtendState = extendingStates::RETRACT;
}

void climber::climberExtend() {
    currentExtendState = extendingStates::SOLEXTEND;
}

void climber::leftClimbToggle() {
    if(!lToggle) {
        m_climberMotorLeft.Set(-0.2);
        lToggle = true;
    } else {
        m_climberMotorLeft.Set(0.0);
        m_climberMotorLeftEncoder.SetPosition(0);
        lToggle = false;
    }
}

void climber::rightClimbToggle() {
    if(!rToggle) {
        m_climberMotorRight.Set(-0.2);
        rToggle = true;
    } else {
        m_climberMotorRight.Set(0.0);
        m_climberMotorRightEncoder.SetPosition(0);
        rToggle = false;
    }
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
        case extendingStates::WAITING:
            if (m_climberMotorLeftEncoder.GetPosition() > 18.9 && m_climberMotorRightEncoder.GetPosition() > 19.9) {
                currentExtendState = extendingStates::CLOSESOLENOIDS;
            }

            climbState = "WAITING";
            break;
        case extendingStates::CLOSESOLENOIDS:

            climbState = "CLOSESOLENOIDS";
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
        default:
        case extendingStates::INIT:

            climbState = "INIT";
            break;
    }
}