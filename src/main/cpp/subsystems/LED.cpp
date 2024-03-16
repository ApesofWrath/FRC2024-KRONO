#pragma once
#include "subsystems/LED.h"

using namespace ctre::phoenix;


LED::LED(CANifier& LEDCanifier) : m_LEDCANifier{LEDCanifier} {
    // Set default behavior to solid white
    setSolid({100.0, 100.0, 100.0});
}

// set color of LED to a single solid color
void LED::setSolid(std::array<double, 3> RGB) {
    ledFunction = [RGB](){
        return RGB;
    };
}

// Set LED color to a blinking color, speed is in seconds
void LED::setBlinking(std::array<double, 3> RGB, double speed) {
    ledFunction = [RGB, speed](){
        auto currentTime = std::chrono::system_clock::now().time_since_epoch();
        double currentSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTime).count();

        std::array<double, 3> result;

        if ( std::fmod(currentSeconds, speed)/speed >= 0.5){
            result = RGB;
        }else{
            result = {0.0, 0.0, 0.0};
        }
        return result;
    };
}



void LED::Periodic() {
    // Get RGB value from lambda function
    std::array<double, 3> RGB = ledFunction();

    m_LEDCANifier.SetLEDOutput(RGB[0], CANifier::LEDChannelB); // NOTE WHEN TESTING: Make sure channels are correct
    m_LEDCANifier.SetLEDOutput(RGB[1], CANifier::LEDChannelA);
    m_LEDCANifier.SetLEDOutput(RGB[2], CANifier::LEDChannelC);

    }

