#include "commands/ShooterIntake/scoreAmp.h" // relevent header file

scoreAmp::scoreAmp(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("scoreAmp"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void scoreAmp::Initialize() { printf("scoreAmp Initialized \n"); } // print debug message on initialization

void scoreAmp::Execute() { // on command call (button press)
    m_intake->scoreAmp();
}

bool scoreAmp::IsFinished() { return true; } // return when ??