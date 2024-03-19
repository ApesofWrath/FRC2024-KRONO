#pragma once
#include "subsystems/LED.h"

using namespace ctre::phoenix;
using namespace generalConstants;

LED::LED(CANifier& LEDCanifier) : m_LEDCANifier{LEDCanifier} {
    // Set default behavior to solid white
    setCycle(1);
}

void LED::setSolid(std::array<double, 3> RGB) {
    auto color = frc::Color(RGB[0], RGB[1], RGB[2]);
    ledFunction = [color](){
        return color;
    };
}
void LED::setSolid(frc::Color color){
    ledFunction = [color](){
        return color;
    };
}

void LED::setBlinking(std::array<double, 3> RGB, double speed) {
    auto color = frc::Color(RGB[0], RGB[1], RGB[2]);
    ledFunction = [color, speed](){
        double currentSeconds = timer.Get().value();

        frc::Color result;

        if ( std::fmod(currentSeconds, 1/speed)*speed >= 0.5){
            result = color;
        }else{
            result = {0.0, 0.0, 0.0};
        }
        return result;
    };
}
void LED::setBlinking(frc::Color color, double speed) {
    ledFunction = [color, speed](){
        double currentSeconds = timer.Get().value();

        frc::Color result;

        if ( std::fmod(currentSeconds, 1/speed)*speed >= 0.5){
            result = color;
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
        frc::Color color = {RGB[0], RGB[1], RGB[2]};
        return color;
    };
}

void LED::setBrightness(double brightness){
    brightness = std::clamp(brightness, 0.0, 1.0)*100;
}

void LED::setBrightness(int percentBrightness){
    brightness = std::clamp(brightness, 0.0, 100.0);

}

void LED::setTeamColor(){
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance == frc::DriverStation::Alliance::kRed){
        setSolid(frc::Color::kRed);
    } else if (alliance == frc::DriverStation::Alliance::kBlue) {
        setSolid(frc::Color::kBlue);        
    } else {
        setSolid(frc::Color::kWhite);
    }

}

void LED::Periodic() {
    // Get RGB value from lambda function
    auto color = ledFunction();

    m_LEDCANifier.SetLEDOutput(color.red*brightness, CANifier::LEDChannelB); // NOTE WHEN TESTING: Make sure channels are correct
    m_LEDCANifier.SetLEDOutput(color.green*brightness, CANifier::LEDChannelA);
    m_LEDCANifier.SetLEDOutput(color.blue*brightness, CANifier::LEDChannelC);

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
            m_LED.setSolid(frc::Color::kOrange);
        
        case intakeshooterStates::SPINUP:
            if (m_intakeshooter.shooterAtSpeed()){
                m_LED.setSolid(frc::Color::kGreen);
            } else {
                m_LED.setBlinking(frc::Color::kYellow, 5.0);
            }
        default:
            m_LED.setTeamColor();
        }
    } else {
        m_LED.setCycle(1.0);
    }
}