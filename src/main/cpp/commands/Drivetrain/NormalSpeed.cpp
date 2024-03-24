#include "commands/Drivetrain/NormalSpeed.h" // relevent header file

NormalSpeed::NormalSpeed(drivetrain* drivetrain) : m_drivetrain{drivetrain} { // constructor for command class
    SetName("NormalSpeed"); // set the ?? name
    AddRequirements({m_drivetrain}); // require the m_drivetrain pointer
}

void NormalSpeed::Initialize() { printf("NormalSpeed Initialized \n"); } // print debug message on initialization

void NormalSpeed::Execute() { // on command call (button press)
    m_drivetrain->normalSpeed(); // sets slowconst variable to 1.0
}

bool NormalSpeed::IsFinished() { return true; } // return when ??