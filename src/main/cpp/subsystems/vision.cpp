#include "subsystems/vision.h"
//Class for vision
vision::vision() : m_frontLimelight(Limelight("frontLimelight")), m_backLimelight(Limelight("backLimelight")) {}

void vision::Periodic() {
}


//Get Robot's Pose
std::vector<double> vision::GetBotPose() {
  std::vector<double> front_limelight_reported_pose = m_frontLimelight.GetBotPose();
  std::vector<double> back_limelight_reported_pose = m_frontLimelight.GetBotPose();
  if (front_limelight_reported_pose == m_emptyVector){
    return back_limelight_reported_pose;
  } else {
    return front_limelight_reported_pose;
  }

}



std::vector<bool> vision::TargetFound() {
  bool front_target_found = m_frontLimelight.TargetFound();
  bool back_target_found = m_backLimelight.TargetFound();
  
  std::vector<bool> result = {front_target_found, back_target_found};
  return result;
}

std::vector<double> vision::GetLatency() {
  double front_latency = m_frontLimelight.TargetFound();
  double back_latency = m_backLimelight.TargetFound();
  
  std::vector<double> result = {front_latency, back_latency};
  return result; 
}

// Takes Robot position and Speaker position and returns the angle from the Robot to the Speaker in radians
double vision::CalculateAngle(std::vector<double> RobotPosition, std::vector<double> AmpLocation){
  double result = atan((AmpLocation[0]-RobotPosition[0])/(AmpLocation[1]-RobotPosition[1]));
  return result;
}