#include "commands/ShooterIntake/intakeActivate.h" // relevent header file

intakeActivate::intakeActivate(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("intakeActivate"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void intakeActivate::Initialize() { printf("intakeActivate Initialized \n"); } // print debug message on initialization

void intakeActivate::Execute() { // on command call (button press)
    m_intake->intakeActivate();
}

bool intakeActivate::IsFinished() { return true; } // return when ??