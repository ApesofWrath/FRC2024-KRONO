#include "commands/ShooterIntake/fire.h" // relevent header file

fire::fire(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("fire"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void fire::Initialize() { 
    printf("fire Initialized \n"); 
    m_intake->fire();
} // print debug message on initialization

void fire::Execute() { // on command call (button press)
    
}

bool fire::IsFinished() { return m_intake->getState() == intakeshooterStates::POSTFIRE; } // return when ??