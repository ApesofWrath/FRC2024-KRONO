#pragma once
#include "subsystems/LED.h"

using namespace ctre::phoenix;
using namespace generalConstants;

LED::LED(CANifier& LEDCanifier) : m_LEDCANifier{LEDCanifier} {
    // Set default behavior to solid white
    setCycle(1);
}

void LED::setSolid(std::array<double, 3> RGB) {
    ledFunction = [RGB](){
        return RGB;
    };
}

void LED::setBlinking(std::array<double, 3> RGB, double speed) {
    ledFunction = [RGB, speed](){
        double currentSeconds = timer.Get().value();


        std::array<double, 3> result;

        if ( std::fmod(currentSeconds, 1/speed)*speed >= 0.5){
            result = RGB;
        }else{
            result = {0.0, 0.0, 0.0};
        }
        return result;
    };
}

void LED::setCycle(double speed){
    ledFunction = [speed](){
        double currentSeconds = timer.Get().value();
        double hue = std::fmod(currentSeconds, 1/speed)*speed*360;
        std::array<double, 3> RGB = MathFunctions::hueToRGB(hue);
        return RGB;
    };
}

void LED::setBrightness(double percentBrightness){
    brightness = std::clamp(percentBrightness, 0.0, 100.0);
}


void LED::Periodic() {
    // Get RGB value from lambda function
    std::array<double, 3> RGB = ledFunction();

    m_LEDCANifier.SetLEDOutput(RGB[0]*brightness, CANifier::LEDChannelB); // NOTE WHEN TESTING: Make sure channels are correct
    m_LEDCANifier.SetLEDOutput(RGB[1]*brightness, CANifier::LEDChannelA);
    m_LEDCANifier.SetLEDOutput(RGB[2]*brightness, CANifier::LEDChannelC);

    }

