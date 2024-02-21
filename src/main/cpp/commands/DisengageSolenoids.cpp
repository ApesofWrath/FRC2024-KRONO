#include "commands/DisengageSolenoids.h" // relevent header file

DisengageSolenoids::DisengageSolenoids(climber* climber) : m_climber{climber} { // constructor for command class
    SetName("DisengageSolenoids"); // set the ?? name
    AddRequirements({m_climber}); // require the m_drivetrain pointer
}

void DisengageSolenoids::Initialize() { printf("DisengageSolenoids Initialized \n"); } // print debug message on initialization

void DisengageSolenoids::Execute() { // on command call (button press)
    m_climber->disengageSolenoids();
}

bool DisengageSolenoids::IsFinished() { return true; } // return when ??