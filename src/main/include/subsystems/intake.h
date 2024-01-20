#pragma once

#include <numbers>
#include <units/length.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/CANSparkMax.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class intake : public frc2::SubsystemBase {
    public:
    intake();
    void IntakeToggle();

    private:
    rev::CANSparkMax m_motor1;
    rev::CANSparkMax m_motor2;
    bool intakeOn = false;
};

