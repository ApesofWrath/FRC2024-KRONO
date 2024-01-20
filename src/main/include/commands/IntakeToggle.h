#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/intake.h"

class IntakeToggle : public frc2::CommandHelper<frc2::Command, IntakeToggle> {
    public:
    explicit IntakeToggle(intake* intake);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    intake* m_intake;
};