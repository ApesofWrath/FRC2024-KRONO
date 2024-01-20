#include "commands/Drivetrain/SlowDown.h" // relevent header file

SlowDown::SlowDown(drivetrain* drivetrain) : m_drivetrain{drivetrain} { // constructor for command class
    SetName("SlowDown"); // set the ?? name
    AddRequirements({m_drivetrain}); // require the m_drivetrain pointer
}

void SlowDown::Initialize() { printf("SlowDown Initialized \n"); } // print debug message on initialization

void SlowDown::Execute() { // on command call (button press)
    m_drivetrain->slowDown(); // sets slowconst variable to slower value
}

bool SlowDown::IsFinished() { return true; } // return when ??