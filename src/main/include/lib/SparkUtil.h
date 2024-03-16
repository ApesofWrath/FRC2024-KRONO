#pragma once

#include <rev/CANSparkBase.h>
#include <rev/REVLibError.h>
#include <frc2/command/PrintCommand.h>
#include <functional>

class SparkUtil{
    public:
    void configure(rev::CANSparkBase* spark, std::vector<std::function<rev::REVLibError()>> config);
    void configure(rev::CANSparkBase* spark, std::function<rev::REVLibError()> config, int attempt);
    rev::REVLibError setInvert(rev::CANSparkBase* spark, bool isInverted);
    private:
};