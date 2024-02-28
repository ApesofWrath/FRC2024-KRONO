#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/CANSparkMax.h>
#include <Constants.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace climberConstants;

/* enum class telescopeStates {
    EXTENDED,
    UNEXTENDED
}; */

enum class zeroingStates {
    INIT,
    NOTZEROED,
    ZEROED,
    MANUALZERO,
    IDLE
};

enum class extendingStates {
    INIT,
    EXTEND,
    WAITING,
    CLOSESOLENOIDS
};

/* enum class retractingStates {
    INIT,
    RETRACT,
    WAITING,
    CLOSESOLENOIDS
}; */

class climber : public frc2::SubsystemBase {
    public:
    climber();
	/* void SetHeight(double height);
    void TelescopeToggle();
    void TelescopeToggle(telescopeStates state); */

    void climberExtend();
    void climberRetract();
    void disengageSolenoids();
    void zeroClimber();
    void motorRetract();
    void Periodic();

    private:
    rev::CANSparkMax m_climberMotorLeft;
    rev::CANSparkMax m_climberMotorRight;
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();
    rev::SparkPIDController m_climberMotorLeftController = m_climberMotorLeft.GetPIDController();
    rev::SparkPIDController m_climberMotorRightController = m_climberMotorRight.GetPIDController();
    rev::SparkRelativeEncoder m_climberMotorLeftEncoder = m_climberMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder m_climberMotorRightEncoder = m_climberMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

    rev::CANSparkMax m_climberSolenoidLeft;
    rev::CANSparkMax m_climberSolenoidRight;

    // telescopeStates currentTelescopeState = telescopeStates::UNEXTENDED;
    zeroingStates currentZeroState = zeroingStates::IDLE; //INIT
    extendingStates currentExtendState = extendingStates::INIT;
    // retractingStates curentRetractState = retractingStates::INIT;

    std::string zeroState = "";
}; 