#include "subsystems/vision.h"

using namespace visionConstants::fieldConstants;
using namespace visionConstants::shootAngleInterpolation;

//Class for vision
vision::vision() : m_frontLimelight(Limelight("frontLimelight")), m_backLimelight(Limelight("backLimelight")) {
    auto alliance = frc::DriverStation::GetAlliance();
    std::vector<double> speaker_position;
    if (alliance == frc::DriverStation::Alliance::kRed){
        speaker_position = kRedSpeakerLocation;
    } else {
        speaker_position = kBlueSpeakerLocation;
    }
}

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
  if(front_distance > back_distance or front_distance == 0.0_m){
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

frc::Pose2d vision::ToPose2d(){
  std::vector<double> robot_position = GetCloserLimelight()->GetBotPose();
  // Get current robot position, return first 2 values as vector
  std::vector<double> positionXY{};
  frc::Pose2d pose2d(units::length::meter_t{robot_position[0]},
                    units::length::meter_t{robot_position[1]}, 
                    frc::Rotation2d({units::angle::radian_t(robot_position[5])}));
  return pose2d;
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
double vision::CalculateAngle(std::vector<double> RobotPosition, std::vector<double> SpeakerLocation){
  double result = atan((SpeakerLocation[0]-RobotPosition[0])/(SpeakerLocation[1]-RobotPosition[1]));
  return result;
  
}

double vision::CalulateShooterAngle(){
  std::vector<double> speaker_position = speaker_position;
  std::vector<double> bot_pose = GetBotPose();
  double distance_from_speaker = sqrt(pow(speaker_position[0]-bot_pose[0], 2)+pow(speaker_position[0]-bot_pose[0], 2));
  double angle = MathFunctions::lerpPoints(distances, angles, distance_from_speaker);
  return angle;
}