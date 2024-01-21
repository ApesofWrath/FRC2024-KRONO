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

    private:
    rev::CANSparkMax m_rollerMotor1;
    rev::CANSparkMax m_rollerMotor2;
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();
    bool intakeOn = true;
};

