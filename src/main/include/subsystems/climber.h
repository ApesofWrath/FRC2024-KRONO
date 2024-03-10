#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/CANSparkMax.h>
#include <Constants.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace climberConstants;

enum class extendingStates {
    INIT,
    SOLEXTEND,
    EXTEND,
    WAITING,
    RETRACT,
    POSTRETRACT
};

class climber : public frc2::SubsystemBase {
    public:
    climber();
	/* void SetHeight(double height);
    void TelescopeToggle();
    void TelescopeToggle(telescopeStates state); */

    void climberExtend();
    void climberRetract();

    void leftClimbToggle();
    void rightClimbToggle();
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

    extendingStates currentExtendState = extendingStates::INIT;

    int solCount = 0;
    std::string climbState = "";

    bool lToggle = false;
    bool rToggle = false;
}; 