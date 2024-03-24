#include "commands/shooterintake/applyVoltage.h" // relevent header file

applyVoltage::applyVoltage(intakeshooter* intake) : m_intake{intake} { // constructor for command class
    SetName("applyVoltage"); // set the ?? name
    AddRequirements({m_intake}); // require the m_drivetrain pointer
}

void applyVoltage::Initialize() { 
    printf("applyVoltage Initialized \n"); 
} // print debug message on initialization

void applyVoltage::Execute() { // on command call (button press)
    m_intake->applyVoltage();
}

bool applyVoltage::IsFinished() { return true; } // return when ??