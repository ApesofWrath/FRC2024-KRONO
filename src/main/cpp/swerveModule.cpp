#include "swerveModule.h"

using namespace drivetrainConstants::calculations;

// Constructor for creating and configuring the swerve module motors
swerveModule::swerveModule(const double module[]) 
    : m_motorDrive(module[0]),
    m_motorTurn(module[1]),
    m_encoderTurn(module[2]) {

    // Sets both the drive motor and the turn motor to be inverted
    m_motorDrive.SetInverted(true);
    m_motorTurn.SetInverted(true);

    // Sets the idle mode of the swerve module motors to brake (Motors brake when not doing anything)
    m_motorDrive.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_motorTurn.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    // Sets current limits for the swerve module motors(TODO: Figure out stator and/or supply limits)
    m_motorDriveCurrentLimitsConfigs
    .WithSupplyCurrentLimit(40);
    m_motorTurnCurrentLimitsConfigs
    .WithSupplyCurrentLimit(20);

    //Sets PID and Feedforward values for swerve module motors(TODO: Tune for Krakens and figure out what feedforward gains we need)
    m_motorDriveSlot0Configs
    .WithKP(0.3)
    .WithKD(0.2)
    .WithKS(0.0)
    .WithKV(1.0/4.6);

    m_motorTurnSlot0Configs
    .WithKP(0.015)
    .WithKD(0.001);

    m_motorDriveFeedbackConfigs
    .WithSensorToMechanismRatio(kFinalDriveRatio);

    m_motorTurnFeedbackConfigs
    .WithRemoteCANcoder(m_encoderTurn);

    //If we get Phoenix Pro, use this, it's better
    /* m_motorTurnFeedbackConfigs 
    .WithFusedCANcoder(m_encoderTurn)
    .WithRotorToSensorRatio(kFinalTurnRatio); */

    m_motorDrive.GetConfigurator().Apply(m_motorDriveCurrentLimitsConfigs);
    m_motorDrive.GetConfigurator().Apply(m_motorDriveFeedbackConfigs);
    m_motorDrive.GetConfigurator().Apply(m_motorDriveSlot0Configs);

    m_motorTurn.GetConfigurator().Apply(m_motorTurnCurrentLimitsConfigs);
    m_motorTurn.GetConfigurator().Apply(m_motorTurnFeedbackConfigs);
    m_motorTurn.GetConfigurator().Apply(m_motorTurnSlot0Configs);
}

// Gets the position of the swerve module drive motor and turn motor
frc::SwerveModulePosition swerveModule::GetPosition() {
    return {units::meter_t{m_motorDrive.GetPosition().GetValueAsDouble() * kWheelCircumference.value()}, frc::Rotation2d(frc::AngleModulus(units::degree_t{m_encoderTurn.GetAbsolutePosition().GetValue()}))};
}

frc::SwerveModuleState swerveModule::GetState() {
    return {units::meters_per_second_t{m_motorDrive.GetVelocity().GetValueAsDouble() * kWheelCircumference.value()}, frc::Rotation2d(frc::AngleModulus(units::degree_t{m_encoderTurn.GetAbsolutePosition().GetValue()}))};
}

// Applies the wanted speed and direction to the turn and drive motors
void swerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
    const auto state = CustomOptimize(
        referenceState, units::degree_t(m_encoderTurn.GetAbsolutePosition().GetValue()));
    
    m_motorDrive.SetControl(m_driveMotorSpeed.WithVelocity(units::turns_per_second_t(state.speed.value() / kWheelCircumference.value())));
    m_motorTurn.SetControl(m_turnMotorAngle.WithPosition(units::turn_t(state.angle.Degrees())));
    
}

// Calculates difference between desired angle and current angle for swerve modules and other values
frc::SwerveModuleState swerveModule::CustomOptimize(const frc::SwerveModuleState& desiredState,
                                                    const frc::Rotation2d& currentAngle) {
    auto modulusAngle{frc::AngleModulus(currentAngle.Degrees())};
    auto optAngle = desiredState.angle;
    auto optSpeed = desiredState.speed;
    
    auto difference = optAngle.Degrees() - modulusAngle;

    if (difference >= 270_deg) {
        difference = difference - 360_deg;
    } else if (difference <= -270_deg) {
        difference = difference + 360_deg;
    }

    if (units::math::abs(difference) > 90_deg) {
        optSpeed = -desiredState.speed;
        if (difference > 0_deg) {
            difference = difference - 180_deg;
        } else {
            difference = difference + 180_deg;
        }
    }
    optAngle = currentAngle.Degrees() + difference;

    return {optSpeed, optAngle};
}
