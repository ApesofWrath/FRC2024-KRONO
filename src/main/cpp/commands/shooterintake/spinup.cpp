#include "commands/shooterintake/spinup.h" // relevent header file

spinup::spinup(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("spinup"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void spinup::Initialize() { printf("spinup Initialized \n"); } // print debug message on initialization

void spinup::Execute() { // on command call (button press)
    m_intake->spinup();
}

bool spinup::IsFinished() { return true; } // return when ??