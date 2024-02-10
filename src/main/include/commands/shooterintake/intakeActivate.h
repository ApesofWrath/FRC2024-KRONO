#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/intakeshooter.h"

class intakeActivate : public frc2::CommandHelper<frc2::Command, intakeActivate> {
    public:
    explicit intakeActivate(intakeshooter* intake);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    intakeshooter* m_intake;
};