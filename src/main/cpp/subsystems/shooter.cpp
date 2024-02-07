#pragma once
#include "subsystems/shooter.h"

shooter::shooter()
: m_shooterMotorLeft(shooterConstants::kMotorShooterLeft, rev::CANSparkMax::MotorType::kBrushless),
m_shooterMotorRight(shooterConstants::kMotorShooterRight, rev::CANSparkMax::MotorType::kBrushless){

    m_shooterMotorLeft.RestoreFactoryDefaults();
    m_shooterMotorRight.RestoreFactoryDefaults();

    m_shooterMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_shooterMotorLeft.SetSmartCurrentLimit(40.0);
    m_shooterMotorRight.SetSmartCurrentLimit(40.0);

    m_shooterMotorRight.Follow(m_shooterMotorLeft, true);

    m_shooterMotorLeftController.SetP(0);
    m_shooterMotorLeftController.SetI(0);
    m_shooterMotorLeftController.SetD(0);
    m_shooterMotorLeftController.SetFF(0);
    m_shooterMotorLeftController.SetOutputRange(-1.0F, 1.0F);
}

void shooter::ShooterToggle(){
    if (shooterOn == true){
        m_shooterMotorLeft.Set(1.0);
    }
    else if (shooterOn == false){
        m_shooterMotorLeft.Set(0.0);
    }
    shooterOn = !shooterOn;
}

void shooter::ShooterOn() {

}

void shooter::ShooterStop() {

}

void shooter::ShooterIdle() {
    
}