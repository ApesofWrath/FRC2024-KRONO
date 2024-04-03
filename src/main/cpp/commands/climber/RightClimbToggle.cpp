#include "commands/RightClimbToggle.h" // relevent header file

RightClimbToggle::RightClimbToggle(climber* climber) : m_climber{climber} { // constructor for command class
    SetName("RightClimbToggle"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void RightClimbToggle::Initialize() { printf("RightClimbToggle Initialized \n"); } // print debug message on initialization

void RightClimbToggle::Execute() { // on command call (button press)
    m_climber->rightClimbToggle();
}

bool RightClimbToggle::IsFinished() { return true; } // return when ??