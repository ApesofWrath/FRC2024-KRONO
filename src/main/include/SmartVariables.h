#include <frc/smartdashboard/SmartDashboard.h>

namespace SmartVariables{
    // A number that can by dynamically changed with Shuffleboard
    class smartSettableNumber {
    public:
        smartSettableNumber(const std::string& name, double defaultValue, bool resetOnStart=true);
        // Get the variables value
        double get();
        
    private:
        const std::string& m_name;
        double m_defaultValue;
    };

}