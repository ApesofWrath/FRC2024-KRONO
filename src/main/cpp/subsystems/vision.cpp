#include "subsystems/vision.h"
//Class for Vision
Vision::Vision()
    : m_frontLimelightNT(nt::NetworkTableInstance::GetDefault().GetTable("frontlimelight")), m_backLimelightNT(nt::NetworkTableInstance::GetDefault().GetTable("limelight")) {}

void Vision::Periodic() {
}


//Get Robot's Pose
std::vector<double> Vision::GetBotPose() {
  double defaultbotpose[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return m_networkTable->GetNumberArray("botpose", std::span<double>(defaultbotpose, 6));
}

frc::Pose2d Vision::ToPose2d() {
  std::vector<double> defaultbotpose;
  if (m_networkTable->GetNumber("tv", 0) != 1.0) {
    return frc::Pose2d(0_m, 0_m, 0_deg);
  }

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
      defaultbotpose = m_networkTable->GetNumberArray("botpose_wpiblue", std::span<const double>());
  } 
  else {
      defaultbotpose = m_networkTable->GetNumberArray("botpose_wpired", std::span<const double>());
  }

  if (defaultbotpose.size() < 6) {
      return frc::Pose2d(0_m, 0_m, 0_deg);
  }
  
  return frc::Pose2d(frc::Translation2d(units::meter_t (defaultbotpose[0]), units::meter_t (defaultbotpose[1])), frc::Rotation2d(units::degree_t (defaultbotpose[5])));
}

bool Vision::TargetFound() {
  return m_networkTable->GetNumber("tv", 0) == 1.0;
}

double Vision::GetLatency() {
  double defaultbotpose[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> botpose = m_networkTable->GetNumberArray("botpose", std::span<double>(defaultbotpose, 7));
  
  return botpose[6];
}