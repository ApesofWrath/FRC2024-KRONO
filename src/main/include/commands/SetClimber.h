#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/climber.h"

class SetClimber : public frc2::CommandHelper<frc2::Command, SetClimber> {
    public:
    explicit SetClimber(climber* climber, double height);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    climber* m_climber;
	double m_height;
};