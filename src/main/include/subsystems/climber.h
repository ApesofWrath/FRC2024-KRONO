#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/CANSparkMax.h>
#include <Constants.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace climberConstants;

class climber : public frc2::SubsystemBase {
    public:
		climber();

		frc2::CommandPtr climberExtend();
		frc2::CommandPtr climberRetract();
		frc2::CommandPtr leftClimbZero();
		frc2::CommandPtr rightClimbZero();

		void Periodic();

    private:
		rev::CANSparkMax m_climberMotorLeft;
		rev::CANSparkMax m_climberMotorRight;
		rev::SparkPIDController m_climberMotorLeftController = m_climberMotorLeft.GetPIDController();
		rev::SparkPIDController m_climberMotorRightController = m_climberMotorRight.GetPIDController();
		rev::SparkRelativeEncoder m_climberMotorLeftEncoder = m_climberMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
		rev::SparkRelativeEncoder m_climberMotorRightEncoder = m_climberMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

		rev::CANSparkMax m_climberSolenoidLeft;
		rev::CANSparkMax m_climberSolenoidRight;

		std::string climbState = "INIT";
}; 