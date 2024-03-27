#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/CANSparkMax.h>
#include <Constants.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace climberConstants;

enum class extendingStates { // Enter condition [id BUTTON] -> action [-> exit condition -> next state]
    INIT, // DEFAULT, transition from EXTEND, transition from WAITING, transition from POSTRETRACT -> Do nothing
    SOLEXTEND, // climberExtend [] -> activate solenoids, set motors -> 4 loops -> EXTEND
    EXTEND, // transition from SOLEXTEND -> run extender motors ->-> INIT
    RETRACT, // climberRetract [] -> deactivate solenoid, retract motors -> motors retracted -> POSTRETRACT
    POSTRETRACT // transition from RETRACT -> set motors ->-> INIT
};

class climber : public frc2::SubsystemBase {
    public:
    climber();

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