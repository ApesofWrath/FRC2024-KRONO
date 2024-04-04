#include "commands/Drivetrain/Drive.h" // relevent header file

#include <frc/MathUtil.h> // math utilities from FRC llibrary

Drive::Drive( // constructor for command class
             drivetrain* drivetrain, // pointer to drivetrain
             std::function<double()> xSpeed, // double for speed (X)
             std::function<double()> ySpeed, // double for speed (Y)
             std::function<double()> zRotation // double for rotation
            )
    : m_drivetrain{drivetrain},   
      m_xSpeed(std::move(xSpeed)),
      m_ySpeed(std::move(ySpeed)),  
      m_zRotation(std::move(zRotation)) {
  SetName("Drive");  // set the ?? name
  AddRequirements({m_drivetrain}); 
}

void Drive::Initialize() { printf("Drive initialized.\n"); } // print debug message on initialization

void Drive::Execute() { // on command call (button press)
  m_drivetrain->SwerveDrive( // make m_drivetrain point to SwerveDrive function while passing the below values
                            -m_ySpeedLimiter.Calculate(frc::ApplyDeadband((m_ySpeed() * m_drivetrain->kslowConst), 0.08)) * drivetrainConstants::calculations::kChassisMaxSpeed, // ??
                            -m_xSpeedLimiter.Calculate(frc::ApplyDeadband((m_xSpeed() * m_drivetrain->kslowConst), 0.08)) * drivetrainConstants::calculations::kChassisMaxSpeed, // ??
                            -m_zRotationLimiter.Calculate(frc::ApplyDeadband((m_zRotation() * m_drivetrain->kslowConst), 0.20)) * drivetrainConstants::calculations::kModuleMaxAngularVelocity, true); // ??
}

void Drive::End(bool interrupted) { printf("**Drive has been interrupted!**\n"); } // print debug message on end when it is inturrupted

bool Drive::IsFinished() { return false; } // return when ??