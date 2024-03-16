#include "subsystems/climber.h"

climber::climber()
: m_climberMotorLeft(kMotorClimberLeft, rev::CANSparkMax::MotorType::kBrushless),
m_climberMotorRight(kMotorClimberRight, rev::CANSparkMax::MotorType::kBrushless), 
m_climberSolenoidLeft(kSolenoidClimberLeft, rev::CANSparkMax::MotorType::kBrushed), 
m_climberSolenoidRight(kSolenoidClimberRight, rev::CANSparkMax::MotorType::kBrushed)
{
    // Left and Right Climber Motors
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
        [this]() {return m_climberMotorLeftEncoder.SetVelocityConversionFactor(((kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);},
        [this]() {return m_climberMotorLeftEncoder.SetPosition(0.2);}
    };

    std::vector<std::function<rev::REVLibError()>> climberMotorRightConfigs = {
        [this]() {return m_climberMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);},
        [this]() {return m_climberMotorRight.SetSmartCurrentLimit(40.0);},
        [this]() {return m_climberMotorRightController.SetP(0.65);},
        [this]() {return m_climberMotorRightController.SetI(0.0004);},
        [this]() {return m_climberMotorRightController.SetD(0.0);},
        [this]() {return m_climberMotorRightController.SetFF(0.0);},
        [this]() {return m_climberMotorRightController.SetOutputRange(-1.0F,1.0F);},
        [this]() {return m_climberMotorRightController.SetIZone(0.25);},
        [this]() {return m_climberMotorRightEncoder.SetPositionConversionFactor((kRotationsToInchTelescoping) * (kTelescopingRatio));},
        [this]() {return m_climberMotorRightEncoder.SetVelocityConversionFactor(((kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);},
        [this]() {return m_climberMotorRightEncoder.SetPosition(0.2);}
    };

    m_sparkUtil.configure(&m_climberMotorLeft, climberMotorLeftConfigs);
    m_sparkUtil.configure(&m_climberMotorRight, climberMotorRightConfigs);
    
    //Left and Right Climber Solenoids
    std::vector<std::function<rev::REVLibError()>> climberSolenoidLeftConfig = {
        [this]() {return m_climberSolenoidLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);},
        [this]() {return m_climberSolenoidLeft.SetSmartCurrentLimit(40.0);}
    };

    std::vector<std::function<rev::REVLibError()>> climberSolenoidRightConfig = {
        [this]() {return m_climberSolenoidRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);},
        [this]() {return m_climberSolenoidRight.SetSmartCurrentLimit(40.0);},
        [this]() {return m_climberSolenoidRight.Follow(m_climberSolenoidLeft, false);}
    };

    m_sparkUtil.configure(&m_climberSolenoidLeft, climberSolenoidLeftConfig);
    m_sparkUtil.configure(&m_climberSolenoidRight, climberSolenoidRightConfig);
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