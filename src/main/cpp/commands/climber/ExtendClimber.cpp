#include "commands/ExtendClimber.h" // relevent header file

ExtendClimber::ExtendClimber(climber* climber) : m_climber{climber} { // constructor for command class
    SetName("ExtendClimber"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void ExtendClimber::Initialize() { printf("ExtendClimber Initialized \n"); } // print debug message on initialization

void ExtendClimber::Execute() { // on command call (button press)
    m_climber->climberExtend();
}

bool ExtendClimber::IsFinished() { return true; } // return when ??