#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/intakeshooter.h"

class fire : public frc2::CommandHelper<frc2::Command, fire> {
    public:
    explicit fire(intakeshooter* intake);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    intakeshooter* m_intake;
};