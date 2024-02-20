#pragma once

#ifndef VISION_H
#define VISION_H

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
#include "limelight.h"
#include "math.h"

class vision : public frc2::SubsystemBase {
 public:
  vision();

  Limelight* GetCloserLimelight();
  void Periodic() override;
  std::vector<double> GetBotPose();
  bool TargetFound();
  double GetLatency();
  double CalculateAngle(std::vector<double> RobotPosition, std::vector<double> AmpLocation);

 private:
  Limelight m_frontLimelight;
  Limelight m_backLimelight;
  std::vector<double> m_emptyVector;

};
#endif // VISION_H