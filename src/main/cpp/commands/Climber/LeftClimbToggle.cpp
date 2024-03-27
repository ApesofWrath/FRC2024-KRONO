#include "commands/Climber/LeftClimbToggle.h" // relevent header file

LeftClimbToggle::LeftClimbToggle(climber* climber) : m_climber{climber} { // constructor for command class
    SetName("LeftClimbToggle"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void LeftClimbToggle::Initialize() { printf("LeftClimbToggle Initialized \n"); } // print debug message on initialization

void LeftClimbToggle::Execute() { // on command call (button press)
    m_climber->leftClimbToggle();
}

bool LeftClimbToggle::IsFinished() { return true; } // return when ??