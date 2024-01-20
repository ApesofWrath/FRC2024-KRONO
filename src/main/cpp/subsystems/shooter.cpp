#pragma once
#include "subsystems/shooter.h"

shooter::shooter()
: m_shooterMotor1(25, rev::CANSparkMax::MotorType::kBrushless),
m_shooterMotor2(35, rev::CANSparkMax::MotorType::kBrushless){

    m_shooterMotor1.RestoreFactoryDefaults();
    m_shooterMotor2.RestoreFactoryDefaults();

    m_shooterMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_shooterMotor1.SetSmartCurrentLimit(40.0);
    m_shooterMotor2.SetSmartCurrentLimit(40.0);

    m_shooterMotor2.Follow(m_shooterMotor1, true);
    }

    void shooter::ShooterToggle(){
        if (shooterOn == false){
            m_shooterMotor1.Set(0.75);
        }
        else if (shooterOn == true){
            m_shooterMotor1.Set(0.0);
        }
        shooterOn = !shooterOn;
    }