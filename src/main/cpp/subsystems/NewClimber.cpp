// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/NewClimber.h"

NewClimber::NewClimber()
: m_climberMotorLeft(kMotorClimberLeft, rev::CANSparkMax::MotorType::kBrushless),
m_climberMotorRight(kMotorClimberRight, rev::CANSparkMax::MotorType::kBrushless), 
m_climberSolenoidLeft(kSolenoidClimberLeft, rev::CANSparkMax::MotorType::kBrushed), 
m_climberSolenoidRight(kSolenoidClimberRight, rev::CANSparkMax::MotorType::kBrushed)
{
    // Left and Right Climber Motors
    m_climberMotorLeft.RestoreFactoryDefaults();
    m_climberMotorRight.RestoreFactoryDefaults();

    m_climberMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberMotorLeft.SetSmartCurrentLimit(40.0);
    m_climberMotorRight.SetSmartCurrentLimit(40.0);

    m_climberMotorLeft.SetInverted(true);

    // PID for Climber Motor Left and Right
    m_climberMotorLeftController.SetP(0.65); //0.15
    m_climberMotorLeftController.SetI(0.0004);
    m_climberMotorLeftController.SetD(0);
    m_climberMotorLeftController.SetFF(0);
    m_climberMotorLeftController.SetOutputRange(-1.0F,1.0F);
    m_climberMotorLeftController.SetIZone(0.25);

    m_climberMotorLeftEncoder.SetPositionConversionFactor((kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorLeftEncoder.SetVelocityConversionFactor(((kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);

    m_climberMotorRightController.SetP(1.05); //0.55
    m_climberMotorRightController.SetI(0.0004);
    m_climberMotorRightController.SetD(0);
    m_climberMotorRightController.SetFF(0);
    m_climberMotorRightController.SetOutputRange(-1.0F,1.0F);
    m_climberMotorRightController.SetIZone(0.25);

    m_climberMotorRightEncoder.SetPositionConversionFactor((kRotationsToInchTelescoping) * (kTelescopingRatio));
    m_climberMotorRightEncoder.SetVelocityConversionFactor(((kRotationsToInchTelescoping) * (kTelescopingRatio)) / 60.0);

    //Left and Right Climber Solenoids
    m_climberSolenoidLeft.RestoreFactoryDefaults();
    m_climberSolenoidRight.RestoreFactoryDefaults();

    m_climberSolenoidLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_climberSolenoidRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_climberSolenoidLeft.SetSmartCurrentLimit(40.0);
    m_climberSolenoidRight.SetSmartCurrentLimit(40.0);


    // Set Climber Positions for Encoders
    m_climberMotorLeftEncoder.SetPosition(0.2);
    m_climberMotorRightEncoder.SetPosition(0.2);

    // Burn all settings
    m_climberMotorLeft.BurnFlash();
    m_climberMotorRight.BurnFlash();
    m_climberSolenoidLeft.BurnFlash();
    m_climberSolenoidRight.BurnFlash();
}

void NewClimber::SetHeight(units::length::meter_t leftHeight, units::length::meter_t rightHeight){
    m_leftDesiredHeight = leftHeight;
    m_rightDesiredHeight = rightHeight;
}

void NewClimber::SetHeight(units::length::meter_t height){
    m_leftDesiredHeight = height;
    m_rightDesiredHeight = height;
}

void NewClimber::setSolenoidState(solenoidStates state){
     if (state == solenoidStates::UNLOCKED && m_solenoidState == solenoidStates::LOCKED){
        m_unlockTime = generalConstants::timer.Get();
     }
    m_solenoidState = state;
}

bool NewClimber::isClimbAllowed(){
    bool climbAllowed;
    if (generalConstants::timer.HasElapsed(m_unlockTime+climberConstants::ktimeToSolenoidUnlock) && m_solenoidState == solenoidStates::UNLOCKED){
        climbAllowed = true;
    } else {
        climbAllowed = false;
    }
    return climbAllowed;
}


// This method will be called once per scheduler run
void NewClimber::Periodic() {
    switch (m_solenoidState)
    {
    case solenoidStates::LOCKED:
        m_climberSolenoidLeft.SetVoltage(ksolenoidLockedVoltage);

        break;
    
    case solenoidStates::UNLOCKED:
        m_climberSolenoidLeft.SetVoltage(ksolenoidUnlockedVoltage);

        break;
    
    default:
        m_climberMotorLeft.SetVoltage(ksolenoidLockedVoltage);
        
        break;
    }

    if (isClimbAllowed()){
        m_climberMotorLeftController.SetReference(m_leftDesiredHeight.value(), rev::CANSparkMax::ControlType::kPosition);
        m_climberMotorRightController.SetReference(m_rightDesiredHeight.value(), rev::CANSparkMax::ControlType::kPosition);
    } else {
        m_climberMotorLeftController.SetReference(0.0, rev::CANSparkMax::ControlType::kVelocity);   
        m_climberMotorRightController.SetReference(0.0, rev::CANSparkMax::ControlType::kVelocity);   
    }

}
