#include "commands/ZeroClimber.h" // relevent header file

ZeroClimber::ZeroClimber(climber* climber) : m_climber{climber} { // constructor for command class
    SetName("ZeroClimber"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void ZeroClimber::Initialize() { printf("ZeroClimber Initialized \n"); } // print debug message on initialization

void ZeroClimber::Execute() { // on command call (button press)
    m_climber->zeroClimber();
}

bool ZeroClimber::IsFinished() { return true; } // return when ??