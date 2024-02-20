#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <Constants.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
using namespace climberConstants;

enum class telescopeStates {
    EXTENDED,
    UNEXTENDED
};

class climber : public frc2::SubsystemBase {
    public:
    climber();
	void SetHeight(double height);
    void TelescopeToggle();
    void TelescopeToggle(telescopeStates state);
    void Periodic();

    private:
    rev::CANSparkMax m_climberMotorLeft;
    rev::CANSparkMax m_climberMotorRight;
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();
    rev::SparkMaxPIDController m_climberMotorLeftController = m_climberMotorLeft.GetPIDController();
    rev::SparkMaxPIDController m_climberMotorRightController = m_climberMotorRight.GetPIDController();
    rev::SparkMaxRelativeEncoder m_climberMotorLeftEncoder = m_climberMotorLeft.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkMaxRelativeEncoder m_climberMotorRightEncoder = m_climberMotorRight.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);

    rev::CANSparkMax m_climberSolenoidLeft;
    rev::CANSparkMax m_climberSolenoidRight;

    telescopeStates currentTelescopeState = telescopeStates::UNEXTENDED;
}; 