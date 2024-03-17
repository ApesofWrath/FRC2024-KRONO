#pragma once

#include <MathFunctions.h>

#include <stdalign.h>
#include <ctre/phoenix/CANifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <algorithm>




class LED : public frc2::SubsystemBase {
    public:
    LED(ctre::phoenix::CANifier& LEDCANifier);
    // Set LED color to a blinking color, speed is in blinks per second
    void setBlinking(std::array<double, 3> RGB, double speed);
    // set color of LED to a single solid color
    void setSolid(std::array<double, 3> RGB);
    // Cycle between all the colors of the rainbow, speed is in full cycles per second
    void setCycle(double speed);
    // Set percent brightness. Values <0 or >100 will be clamped
    void setBrightness(double percentBrightness);
    private:
    ctre::phoenix::CANifier& m_LEDCANifier;
    void Periodic();
    double brightness;
    // This lambda function is called to get the RGB values that the LEDs should be set to, and can be dynamically swapped with different lambda fucntions for different funcitonality
    std::function<std::array<double, 3>()> ledFunction;
};
