#include "commands/shooterintake/fireSPEAKER.h" // relevent header file

fireSPEAKER::fireSPEAKER(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("fireSPEAKER"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void fireSPEAKER::Initialize() { printf("fireSPEAKER Initialized \n"); } // print debug message on initialization

void fireSPEAKER::Execute() { // on command call (button press)
    m_intake->fireSPEAKER();
}

bool fireSPEAKER::IsFinished() { return true; } // return when ??