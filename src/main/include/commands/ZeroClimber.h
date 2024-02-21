#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/climber.h"

class ZeroClimber : public frc2::CommandHelper<frc2::Command, ZeroClimber> {
    public:
    explicit ZeroClimber(climber* climber);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    climber* m_climber;
};