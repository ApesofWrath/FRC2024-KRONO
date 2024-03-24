// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/vision.h"



vision::vision() : m_networkTable(nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")){

}

// This method will be called once per scheduler run
void vision::Periodic() {
}

double vision::getHeadingError(){
    double tx = m_networkTable->GetNumber("tx", 0.0);

    float heading_error = -tx;
    float steering_adjust = 0.0f;
    if (abs(heading_error) > 1.0) 
    {
        if (heading_error < 0) 
        {
            steering_adjust = visionConstants::Kp*heading_error + visionConstants::min_command;
        } 
        else 
        {
            steering_adjust = visionConstants::Kp*heading_error - visionConstants::min_command;
        }
    } 
    return -steering_adjust;
}
