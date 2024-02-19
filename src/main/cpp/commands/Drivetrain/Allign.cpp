#include "commands/Drivetrain/Allign.h" // relevent header file
using namespace visionConstants::fieldConstants;

Allign::Allign(drivetrain* drivetrain, vision* vision) : m_drivetrain{drivetrain}, m_vision{vision} { // constructor for command class
    SetName("Allign"); // set the ?? name
    AddRequirements({m_drivetrain}); // require the m_drivetrain pointer
}

void Allign::Initialize() { printf("Allign Initialized \n"); } // print debug message on initialization

void Allign::Execute() {
    // Get Robot's XY Position
    std::vector<double> robot_pose = m_vision->GetBotPose();
    std::vector<double> robot_position{robot_pose[0], robot_pose[1]};

    // Set speaker_position based on the alliance
    auto alliance = frc::DriverStation::GetAlliance();
    std::vector<double> speaker_position;
    if (alliance = frc::DriverStation::Alliance::kRed){
        std::vector<double> speaker_postion = kRedSpeakerLocation;
    } else {
        std::vector<double> speaker_postion = kBlueSpeakerLocation;
    }

    // Calculate desired angle 
    double desired_angle = m_vision->CalculateAngle(robot_position, speaker_position);

    

}

bool Allign::IsFinished() { return true; } // return when ??