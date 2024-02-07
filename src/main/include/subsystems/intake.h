#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <Constants.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class intake : public frc2::SubsystemBase {
    public:
    intake();
    void IntakeToggle();
    void IntakeOn();
    void IntakeStop();
    void IntakeIdle();

    private:
    rev::CANSparkMax m_intakeMotorLeft;
    rev::CANSparkMax m_intakeMotorRight;
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();

    rev::SparkMaxPIDController m_intakeMotorLeftController = m_intakeMotorLeft.GetPIDController();
    rev::SparkMaxPIDController m_intakeMotorRightController = m_intakeMotorRight.GetPIDController();

    bool intakeOn = true;
};

