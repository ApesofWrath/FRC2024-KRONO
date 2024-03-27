#pragma once
#include "subsystems/LED.h"

using namespace ctre::phoenix;
using namespace generalConstants;

LED::LED(CANifier& LEDCanifier) : m_LEDCANifier{LEDCanifier} {
    // Set default behavior to solid white
    setBrightness(100);
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

void LED::setBrightness(double setBrightness){
    brightness = std::clamp(setBrightness, 0.0, 1.0)*100;
}

void LED::setBrightness(int percentBrightness){
    brightness = std::clamp(percentBrightness, 0, 100);

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
    m_LEDCANifier.SetLEDOutput(color.green*brightness, CANifier::LEDChannelC);
    m_LEDCANifier.SetLEDOutput(color.blue*brightness, CANifier::LEDChannelA);

    }

LEDmanager::LEDmanager(LED& LED, intakeshooter& intakeshooter) : m_LED{LED}, m_intakeshooter{intakeshooter} {
    frc::SmartDashboard::PutNumber("LED Brightness", 1.0);
}


void LEDmanager::Periodic(){
    double brightness =  frc::SmartDashboard::GetNumber("LED Brightness", 1.0)/100;
    m_LED.setBrightness(brightness);
    if (frc::DriverStation::IsEnabled()){
        intakeshooterStates intakeshooterState = m_intakeshooter.getState();
        switch (intakeshooterState) {

        case intakeshooterStates::IDLE:
            m_LED.setTeamColor();

            break;
        case intakeshooterStates::BACKOFF:
            m_LED.setSolid(frc::Color::kYellow);

            break;
        case intakeshooterStates::NOTEFORWARD:
            m_LED.setSolid(frc::Color::kYellow);

            break;
        case intakeshooterStates::HOLDING:
            m_LED.setSolid(frc::Color::kTeal);

            break;
        case intakeshooterStates::SPINUPPIGEON:
            if (m_intakeshooter.shooterAtSpeed()){
                m_LED.setSolid(frc::Color::kGreen);
            } else {
                m_LED.setBlinking(frc::Color::kOrange, 5.0);
            }

            break;
        default:
            m_LED.setTeamColor();

            break;
        }
    } else {
        m_LED.setCycle(1.0);
    }
}