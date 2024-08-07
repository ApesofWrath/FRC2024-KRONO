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

frc2::CommandPtr climber::climberRetract() {
    return frc2::cmd::Run([this]{
		climbState = "RETRACT";
		m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(0));
		m_climberMotorLeftController.SetReference(0.5, rev::CANSparkMax::ControlType::kPosition);
		m_climberMotorRightController.SetReference(0.5, rev::CANSparkMax::ControlType::kPosition);
	}).Until([this]{
		return m_climberMotorLeftEncoder.GetPosition() < 0.8 && m_climberMotorRightEncoder.GetPosition() < 0.8;
	}).AndThen([this]{
		m_climberMotorLeft.Set(0);
		m_climberMotorRight.Set(0);
		climbState = "INIT";
	});
}

frc2::CommandPtr climber::climberExtend() {
    return frc2::cmd::Run([this]{
		climbState = "EXTEND";
		m_climberSolenoidLeft.SetVoltage(units::voltage::volt_t(12));
	}).WithTimeout(0.1_s).AndThen([this]{
		m_climberMotorLeft.Set(-0.05);
		m_climberMotorRight.Set(-0.05);
	}).WithTimeout(0.1_s).AndThen([this]{
		m_climberMotorLeftController.SetReference(19, rev::CANSparkMax::ControlType::kPosition);
		m_climberMotorRightController.SetReference(20, rev::CANSparkMax::ControlType::kPosition);
		climbState = "INIT";
	});
}

frc2::CommandPtr climber::leftClimbZero() {
    return frc2::cmd::RunOnce([this]{
			m_climberMotorLeft.Set(-0.2);
	}).AndThen(frc2::cmd::Idle()).FinallyDo([this]{
			m_climberMotorLeft.Set(0.0);
			m_climberMotorLeftEncoder.SetPosition(0);
	});
}

frc2::CommandPtr climber::rightClimbZero() {
	return frc2::cmd::RunOnce([this]{
			m_climberMotorRight.Set(-0.2);
	}).AndThen(frc2::cmd::Idle()).FinallyDo([this]{
			m_climberMotorRight.Set(0.0);
			m_climberMotorRightEncoder.SetPosition(0);
	});
}

void climber::Periodic(){
    frc::SmartDashboard::PutNumber("Tele Pos Left", m_climberMotorLeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Tele Pos Right", m_climberMotorRightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climb Out Curr L", m_climberMotorLeft.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Climb Out Curr R", m_climberMotorRight.GetOutputCurrent());
    frc::SmartDashboard::PutString("Climb State", climbState);
}