#include <lib/SparkUtil.h>

void SparkUtil::configure(rev::CANSparkBase* spark, std::vector<std::function<rev::REVLibError()>> config) {
    configure(spark, [spark]() {return spark->RestoreFactoryDefaults();}, 1);
    configure(spark, [spark]() {return spark->SetCANTimeout(50);}, 1);
    for (unsigned i = 0; i < config.size(); i++) {
        std::function<rev::REVLibError()> func = config.at(i);
        configure(spark, func, 1);
    }
    configure(spark, [spark]() {return spark->SetCANTimeout(20);}, 1);
    spark->BurnFlash();
}

void SparkUtil::configure(rev::CANSparkBase* spark, std::function<rev::REVLibError()> config, int attempt) {
    if (attempt >= 3) {
        frc2::PrintCommand("failed to configure");
        return;
    } 
    if (attempt >= 1) {
        frc2::PrintCommand("attempt for setting configuration failed");
    }
    rev::REVLibError error = config();
    if (error != rev::REVLibError::kOk) {
        configure(spark, config, attempt + 1);
    }
}
rev::REVLibError SparkUtil::setInvert(rev::CANSparkBase* spark, bool isInverted) {
    spark->SetInverted(isInverted);
    return spark->GetLastError();
}