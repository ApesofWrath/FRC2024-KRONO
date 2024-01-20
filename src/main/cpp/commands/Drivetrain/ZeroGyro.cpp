#include "commands/Drivetrain/ZeroGyro.h" // relevent header file

ZeroGyro::ZeroGyro(drivetrain* drivetrain) : m_drivetrain{drivetrain} { // constructor for command class
    SetName("ZeroGyro"); // set the ?? name
    AddRequirements({m_drivetrain}); // require the m_drivetrain pointer
}

void ZeroGyro::Initialize() { printf("ZeroGyro Initialized \n"); } // print debug message on initialization

void ZeroGyro::Execute() { // on command call (button press)
    m_drivetrain->resetGyro();
    // m_drivetrain->ResetOdometry(m_drivetrain->GetOdometry()); // run resetGyro function in drivetrain
}

bool ZeroGyro::IsFinished() { return true; } // return when ??