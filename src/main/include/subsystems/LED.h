#pragma once

#include <stdalign.h>
#include <ctre/phoenix/CANifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <Constants.h>



class LED : public frc2::SubsystemBase {
    public:
    LED(ctre::phoenix::CANifier& LEDCANifier);
    void setBlinking(std::array<double, 3> RGB, double speed);
    void setSolid(std::array<double, 3> RGB);
    private:
    ctre::phoenix::CANifier& m_LEDCANifier;
    void Periodic();

    // This lambda function is called to get the RGB values that the LEDs should be set to, and can be dynamically swapped with different lambda fucntions for different funcitonality
    std::function<std::array<double, 3>()> ledFunction;
};
