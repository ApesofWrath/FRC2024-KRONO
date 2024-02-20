#include "subsystems/vision.h"
//Class for vision
vision::vision() : m_frontLimelight(Limelight("frontLimelight")), m_backLimelight(Limelight("backLimelight")) {}

void vision::Periodic() {
}

Limelight* vision::GetCloserLimelight(){

  // Get Distance from apriltag for Front Limelight
  std::vector<double> front_apriltag_pose = m_frontLimelight.GetTargetPoseRobotSpace();
  units::length::meter_t front_distance{sqrt(pow(front_apriltag_pose[0], 2)+pow(front_apriltag_pose[0], 2)+pow(front_apriltag_pose[0], 1))};


  // Get Distance from apriltag for Back Limelight
  std::vector<double> back_apriltag_pose = m_backLimelight.GetTargetPoseRobotSpace();
  units::length::meter_t back_distance{sqrt(pow(back_apriltag_pose[0], 2)+pow(back_apriltag_pose[0], 2)+pow(back_apriltag_pose[0], 1))};

  // Return pointer to the closer Limelight
  Limelight* selectedLimelight = nullptr;
  if(front_distance > back_distance){
      selectedLimelight = &m_backLimelight;
  } else{
      selectedLimelight = &m_frontLimelight;
  }
  return selectedLimelight;
  

}

//Get Robot's Pose
std::vector<double> vision::GetBotPose() {
  std::vector<double> pose = GetCloserLimelight()->GetBotPose();
  return pose;
}



bool vision::TargetFound() {
  bool target_found = GetCloserLimelight()->TargetFound();
  return target_found;
}

double vision::GetLatency() {
  double latency = GetCloserLimelight()->TargetFound();
  return latency; 
}

// Takes Robot position and Speaker position and returns the angle from the Robot to the Speaker in radians
double vision::CalculateAngle(std::vector<double> RobotPosition, std::vector<double> AmpLocation){
  double result = atan((AmpLocation[0]-RobotPosition[0])/(AmpLocation[1]-RobotPosition[1]));
  return result;
}