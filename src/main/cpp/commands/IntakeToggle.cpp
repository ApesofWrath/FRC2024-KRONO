#include "commands/IntakeToggle.h" // relevent header file

IntakeToggle::IntakeToggle(intake* intake) : m_intake{intake} { // constructor for command class
    SetName("IntakeToggle"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void IntakeToggle::Initialize() { printf("IntakeToggle Initialized \n"); } // print debug message on initialization

void IntakeToggle::Execute() { // on command call (button press)
    m_intake->IntakeToggle();
}

bool IntakeToggle::IsFinished() { return true; } // return when ??