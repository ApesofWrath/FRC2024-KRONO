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
    void ShooterOn();
    void ShooterStop();
    void ShooterIdle();

    private:
    rev::CANSparkMax m_shooterMotorLeft;
    rev::CANSparkMax m_shooterMotorRight;
    //rev::SparkMaxPIDController m_rollerMotor1Controller = m_rollerMotor1.GetPIDController();

    rev::SparkMaxPIDController m_shooterMotorLeftController = m_shooterMotorLeft.GetPIDController();
    rev::SparkMaxPIDController m_shooterMotorRightController = m_shooterMotorRight.GetPIDController();
    bool shooterOn = true;
};

