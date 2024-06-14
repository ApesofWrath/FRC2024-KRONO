#include "commands/ShooterIntake/spinup.h" // relevent header file

spinup::spinup(intakeshooter* intake) : m_intake{intake} { // constructor for command class using vision angle
    SetName("spinup"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

spinup::spinup(intakeshooter* intake, double angle) : m_intake{intake} { // constructor for command class using explicit angle
    SetName("spinup"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
    shootAngle = angle;
}

void spinup::Initialize() { 
    printf("spinup Initialized \n");
    m_intake->spinup(shootAngle);
} // print debug message on initialization

void spinup::Execute() { // on command call (button press)
}

bool spinup::IsFinished() { return (m_intake->shooterAtSpeed() && m_intake->getState() == intakeshooterStates::SPINUPPIGEON) || !m_intake->allowSpinup;} // return when ??