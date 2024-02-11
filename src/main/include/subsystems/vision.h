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

class Vision : public frc2::SubsystemBase {
 public:
  Vision();


  void Periodic() override;


 private:
  std::shared_ptr<nt::NetworkTable> m_frontLimelightNT;
  std::shared_ptr<nt::NetworkTable> m_backLimelightNT;

};
#endif // VISION_H