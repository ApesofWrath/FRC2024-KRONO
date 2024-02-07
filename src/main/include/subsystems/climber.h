#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <Constants.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class climber : public frc2::SubsystemBase {
    public:
    climber();
    // void ClimberRetract();
    // void ClimberExtend();
	void SetHeight(double height);

    private:
    rev::CANSparkMax m_climberMotorLeft;
    rev::CANSparkMax m_climberMotorRight;
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();
    rev:: SparkMaxPIDController m_climberMotorLeftController = m_climberMotorLeft.GetPIDController();
    rev:: SparkMaxPIDController m_climberMotorRightController = m_climberMotorRight.GetPIDController();

};