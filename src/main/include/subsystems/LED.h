#pragma once

#include <MathFunctions.h>
#include <Constants.h>
#include <SmartVariables.h>

#include <stdalign.h>
#include <ctre/phoenix/CANifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <algorithm>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <subsystems/intakeshooter.h>
#include <frc/util/Color.h>
#include <string.h>



class LED : public frc2::SubsystemBase {
    public:
    LED(ctre::phoenix::CANifier& LEDCANifier);

    // Set LED color to a blinking color, speed is in blinks per second
    void setBlinking(std::array<double, 3> RGB, double speed);
    void setBlinking(frc::Color color, double speed);
    // set color of LED to a single solid color
    void setSolid(std::array<double, 3> RGB);
    void setSolid(frc::Color color);

    // Cycle between all the colors of the rainbow, speed is in full cycles per second
    void setCycle(double speed);

    // Set brightness. Values <0 or >1 will be clamped
    void setBrightness(double Brightness);

    // Set percent brightness. Values <0 or >100 will be clamped
    void setBrightness(int percentBrightness);
    void setTeamColor();


    private:
    ctre::phoenix::CANifier& m_LEDCANifier;
    void Periodic();
    SmartVariables::smartSettableNumber m_brightness{"LED Brightness", 100.0};
    // This lambda function is called to get the RGB values that the LEDs should be set to, and can be dynamically swapped with different lambda fucntions for different funcitonality
    std::function<frc::Color()> ledFunction;
};

class LEDmanager : public frc2::SubsystemBase{
    public:
    LEDmanager(LED& LED, intakeshooter& intakeshooter);

    private:
    void Periodic();
    LED& m_LED;
    intakeshooter& m_intakeshooter;
};