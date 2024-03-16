#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/climber.h"

class LeftClimbToggle : public frc2::CommandHelper<frc2::Command, LeftClimbToggle> {
    public:
    explicit LeftClimbToggle(climber* climber);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    climber* m_climber;
};