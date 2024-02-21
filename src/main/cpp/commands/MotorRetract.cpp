#include "commands/MotorRetract.h" // relevent header file

MotorRetract::MotorRetract(climber* climber) : m_climber{climber} { // constructor for command class
    SetName("MotorRetract"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void MotorRetract::Initialize() { printf("MotorRetract Initialized \n"); } // print debug message on initialization

void MotorRetract::Execute() { // on command call (button press)
    m_climber->motorRetract();
}

bool MotorRetract::IsFinished() { return true; } // return when ??