#include "commands/ShooterToggle.h" // relevent header file

ShooterToggle::ShooterToggle(shooter* shooter) : m_shooter{shooter} { // constructor for command class
    SetName("IntakeToggle"); // set the ?? name
    AddRequirements({m_shooter}); // require the m_drivetrain pointer
}

void ShooterToggle::Initialize() { printf("IntakeToggle Initialized \n"); } // print debug message on initialization

void ShooterToggle::Execute() { // on command call (button press)
    m_shooter->ShooterToggle();
}

bool ShooterToggle::IsFinished() { return true; } // return when ??