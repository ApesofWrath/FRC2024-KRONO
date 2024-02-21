#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/climber.h"

class RetractClimber : public frc2::CommandHelper<frc2::Command, RetractClimber> {
    public:
    explicit RetractClimber(climber* climber);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    climber* m_climber;
};