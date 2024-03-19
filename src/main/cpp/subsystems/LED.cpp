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

void LED::setTeamColor(){
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance == frc::DriverStation::Alliance::kRed){
        setSolid({1.0, 0.0, 0.0});
    } else if (alliance == frc::DriverStation::Alliance::kBlue) {
        setSolid({0.0, 0.0, 1.0});        
    } else {
        setSolid({1.0, 1.0, 1.0});
    }

}

void LED::Periodic() {
    // Get RGB value from lambda function
    std::array<double, 3> RGB = ledFunction();

    m_LEDCANifier.SetLEDOutput(RGB[0]*brightness, CANifier::LEDChannelB); // NOTE WHEN TESTING: Make sure channels are correct
    m_LEDCANifier.SetLEDOutput(RGB[1]*brightness, CANifier::LEDChannelA);
    m_LEDCANifier.SetLEDOutput(RGB[2]*brightness, CANifier::LEDChannelC);

    }

 LEDmanager::LEDmanager(LED& LED, intakeshooter& intakeshooter) : m_LED{LED}, m_intakeshooter{intakeshooter} {

}

void LEDmanager::Periodic(){
    if (frc::DriverStation::IsEnabled){
        auto intakeshooterState = m_intakeshooter.getState();
        switch (intakeshooterState) {
        case intakeshooterStates::IDLE:
            m_LED.setTeamColor();
            break;
        case intakeshooterStates::HOLDING:
            m_LED.setSolid({0.988235294118, 0.498039215686, 0.0117647058824});
        default:
            m_LED.setTeamColor();
        }
    } else {
        m_LED.setCycle(1.0);
    }
}