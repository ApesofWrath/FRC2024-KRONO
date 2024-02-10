#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/intakeshooter.h"

class fireSPEAKER : public frc2::CommandHelper<frc2::Command, fireSPEAKER> {
    public:
    explicit fireSPEAKER(intakeshooter* intake);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    intakeshooter* m_intake;
};