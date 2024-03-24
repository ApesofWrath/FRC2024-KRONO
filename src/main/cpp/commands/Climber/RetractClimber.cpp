#include "commands/Climber/RetractClimber.h" // relevent header file

RetractClimber::RetractClimber(climber* climber) : m_climber{climber} { // constructor for command class
    SetName("RetractClimber"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void RetractClimber::Initialize() { printf("RetractClimber Initialized \n"); } // print debug message on initialization

void RetractClimber::Execute() { // on command call (button press)
    m_climber->climberRetract();
}

bool RetractClimber::IsFinished() { return true; } // return when ??