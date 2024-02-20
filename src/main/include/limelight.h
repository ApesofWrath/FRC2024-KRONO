#pragma once

#ifndef Limelight_H
#define Limelight_H

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/SpanExtras.h"
#include <span>

// #include <spanstream>

class Limelight : public frc2::SubsystemBase {
 public:
  Limelight(std::string name);


  void Periodic() override;
  double GetTargetX();
  void SelectPipeline(int id);
  std::vector<double> GetBotPose();
  std::vector<double> GetTargetPoseRobotSpace();
  std::vector<double> GetRobotPoseTargetSpace();
  frc::Pose2d ToPose2d();
  bool TargetFound();
  double GetLatency();

 private:
  std::shared_ptr<nt::NetworkTable> m_networkTable;

};
#endif // Limelight_H