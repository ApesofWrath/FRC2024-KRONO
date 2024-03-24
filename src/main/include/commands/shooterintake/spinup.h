#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/intakeshooter.h"

class spinup : public frc2::CommandHelper<frc2::Command, spinup> {
    public:
    explicit spinup(intakeshooter* intake);
    explicit spinup(intakeshooter* intake, double angle);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    private:
    intakeshooter* m_intake;
    double shootAngle = NAN; // explicit shooter angle, used for non-vision version
};