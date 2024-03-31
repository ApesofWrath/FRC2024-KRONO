#include <SmartVariables.h>

SmartVariables::smartSettableNumber::smartSettableNumber(const std::string& name, double defaultValue, bool resetOnStart) : m_name{name}, m_defaultValue{defaultValue} {
    if (resetOnStart){
        frc::SmartDashboard::PutNumber(name, m_defaultValue);
    } else {
        frc::SmartDashboard::SetDefaultNumber(name, m_defaultValue);
    }
}

double SmartVariables::smartSettableNumber::get(){
    return frc::SmartDashboard::GetNumber(m_name, m_defaultValue);
}