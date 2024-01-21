#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <Constants.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class shooter : public frc2::SubsystemBase {
    public:
    shooter();
    void ShooterToggle();

    private:
    rev::CANSparkMax m_shooterMotor1;
    rev::CANSparkMax m_shooterMotor2;
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();
    bool shooterOn = true;
};

