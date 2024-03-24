#include "commands/ShooterIntake/intakeRetract.h" // relevent header file

intakeRetract::intakeRetract(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("intakeRetract"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void intakeRetract::Initialize() { printf("intakeRetract Initialized \n"); } // print debug message on initialization

void intakeRetract::Execute() { // on command call (button press)
    m_intake->intakeRetract();
}

bool intakeRetract::IsFinished() { return true; } // return when ??