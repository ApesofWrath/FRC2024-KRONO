#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/shooter.h"

class ShooterToggle : public frc2::CommandHelper<frc2::Command, ShooterToggle> {
    public:
    explicit ShooterToggle(shooter* shooter);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    shooter* m_shooter;
};