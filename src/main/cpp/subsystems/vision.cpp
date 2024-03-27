// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/vision.h"



vision::vision() : m_networkTable(nt::NetworkTableInstance::GetDefault().GetTable("limelight-front")){

}

// This method will be called once per scheduler run
void vision::Periodic() {
    double distance = getDistance();
    frc::SmartDashboard::PutNumber("Apriltag distance", distance);

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

double vision::getDistance(){
    double targetOffsetAngle_Vertical = m_networkTable->GetNumber("ty",0.0);
    auto pose = m_networkTable->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
    double distance_m = sqrt(pow(pose[0], 2)+pow(pose[2], 2));
    double distance_in = distance_m*39.37;
    return distance_in;
    // how many degrees back is your limelight rotated from perfectly vertical?
    // double limelightMountAngleDegrees = 40.0; 

    // // distance from the center of the Limelight lens to the floor
    // double limelightLensHeightInches = 13.75; 

    // // distance from the target to the floor
    // double goalHeightInches = 57.13; 

    // double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // //calculate distance
    // double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/tan(angleToGoalRadians);
    // return distanceFromLimelightToGoalInches;
}
