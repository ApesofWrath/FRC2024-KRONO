#include "commands/shooterintake/rapidFire.h" // relevent header file

rapidFire::rapidFire(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("rapidFire"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void rapidFire::Initialize() { printf("rapidFire Initialized \n"); } // print debug message on initialization

void rapidFire::Execute() { // on command call (button press)
    m_intake->rapidFire();
}

bool rapidFire::IsFinished() { return true; } // return when ??