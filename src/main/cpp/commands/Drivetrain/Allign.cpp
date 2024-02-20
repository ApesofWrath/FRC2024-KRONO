#include "commands/Drivetrain/Allign.h" // relevent header file
using namespace visionConstants::fieldConstants;

Allign::Allign(drivetrain* drivetrain, vision* vision) : m_drivetrain{drivetrain}, m_vision{vision} { // constructor for command class
    SetName("Allign"); // set the ?? name
    AddRequirements({m_drivetrain}); // require the m_drivetrain pointer
    
    // Set speaker_position based on the alliance
    auto alliance = frc::DriverStation::GetAlliance();
    std::vector<double> speaker_position;
    if (alliance == frc::DriverStation::Alliance::kRed){
        speaker_position = kRedSpeakerLocation;
    } else {
        speaker_position = kBlueSpeakerLocation;
    }
}


void Allign::Initialize() { printf("Allign Initialized \n"); } // print debug message on initialization

void Allign::Execute() {
    // Get Robot's XY Position
    std::vector<double> robot_pose = m_vision->GetBotPose();
    std::vector<double> robot_position{robot_pose[0], robot_pose[1]};
    double robot_z_rotation = robot_pose[5];


    // Calculate desired angle 
    double desired_angle = m_vision->CalculateAngle(robot_position, speaker_position); // Note: CHECK WHAT UNITS LIMELIGHT USES FOR ANGLE
    units::angular_velocity::radians_per_second_t desired_speed{m_PIDController.Calculate(robot_z_rotation, desired_angle)};
    m_drivetrain->SwerveDrive(0_mps, 0_mps, units::angular_velocity::radians_per_second_t(desired_angle), true);
}

bool Allign::IsFinished() { return true; } // return when ??