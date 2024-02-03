#include "commands/SetClimber.h" // relevent header file

SetClimber::SetClimber(climber* climber, double height) : m_climber{climber}, m_height{height} { // constructor for command class
    SetName("SetClimber"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void SetClimber::Initialize() { printf("SetClimber Initialized \n"); } // print debug message on initialization

void SetClimber::Execute() { // on command call (button press)
    m_climber->SetHeight(m_height);
}

bool SetClimber::IsFinished() { return true; } // return when ??