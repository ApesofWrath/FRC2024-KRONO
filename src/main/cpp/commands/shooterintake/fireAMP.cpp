#include "commands/shooterintake/fireAMP.h" // relevent header file

fireAMP::fireAMP(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("fireAMP"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void fireAMP::Initialize() { printf("fireAMP Initialized \n"); } // print debug message on initialization

void fireAMP::Execute() { // on command call (button press)
    m_intake->fireAMP();
}

bool fireAMP::IsFinished() { return true; } // return when ??