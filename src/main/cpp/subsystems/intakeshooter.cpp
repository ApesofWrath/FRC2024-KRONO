#pragma once
#include "subsystems/intakeshooter.h"
using namespace shooterConstants;
using namespace intakeConstants;
using namespace generalConstants;

intakeshooter::intakeshooter(frc2::CommandXboxController* controllerMain, frc2::CommandXboxController* controllerOperator, ctre::phoenix::CANifier& beambreakCanifier)
: m_intakeMotorLeft(kIntakeMotorLeft),
m_intakeMotorRight(kIntakeMotorRight),
m_shooterMotorLeft(kMotorShooterLeft, rev::CANSparkMax::MotorType::kBrushless),
m_shooterMotorRight(kMotorShooterRight, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(kIntakeRotationMotor, rev::CANSparkMax::MotorType::kBrushless),
m_BeambreakCanifier(beambreakCanifier),
m_Pigeon(kPigeon),
m_controllerMain(controllerMain),
m_controllerOperator(controllerOperator)
{
    //Kraken Settings
    m_intakeMotorRight.SetControl(ctre::phoenix6::controls::Follower(kIntakeMotorLeft, false));
    motorOutputConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

    currentLimitsConfigs.WithStatorCurrentLimitEnable(true);
    currentLimitsConfigs.WithStatorCurrentLimit(40.0);

    // Kraken PID Values
    slotZeroConfigs.WithKP(0.008);
    slotZeroConfigs.WithKI(0.0);
    slotZeroConfigs.WithKD(0.0);
    slotZeroConfigs.WithKV(12.0 / 1000.0);

    m_intakeMotorLeft.GetConfigurator().Apply(motorOutputConfigs);
    m_intakeMotorLeft.GetConfigurator().Apply(currentLimitsConfigs);
    m_intakeMotorLeft.GetConfigurator().Apply(slotZeroConfigs);

    m_intakeMotorRight.GetConfigurator().Apply(motorOutputConfigs);
    m_intakeMotorRight.GetConfigurator().Apply(currentLimitsConfigs);
    m_intakeMotorRight.GetConfigurator().Apply(slotZeroConfigs);

    m_velocityIntake.WithSlot(0);
    m_velocityIntake.WithEnableFOC(false);
    
    //Neos
    m_shooterMotorLeft.RestoreFactoryDefaults();
    m_shooterMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterMotorLeft.SetSmartCurrentLimit(40.0);

    m_shooterMotorRight.RestoreFactoryDefaults();
    m_shooterMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterMotorRight.SetSmartCurrentLimit(40.0);
    
    // rotation motor
    m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rotationMotor.SetSmartCurrentLimit(80.0);

    // shooter motor controllers (left and right)
    m_shooterMotorLeftController.SetP(0.0003);
    m_shooterMotorLeftController.SetI(0);
    m_shooterMotorLeftController.SetD(0);
    m_shooterMotorLeftController.SetFF(1.0 / 5035.0);
    m_shooterMotorLeftController.SetOutputRange(-1.0F, 1.0F);

    m_shooterLeftEncoder.SetMeasurementPeriod(8.0);
    m_shooterLeftEncoder.SetAverageDepth(1.0);

    m_shooterMotorRightController.SetP(0.0003);
    m_shooterMotorRightController.SetI(0);
    m_shooterMotorRightController.SetD(0);
    m_shooterMotorRightController.SetFF(1.0 / 5035.0);
    m_shooterMotorRightController.SetOutputRange(-1.0F, 1.0F);

    m_shooterRightEncoder.SetMeasurementPeriod(8.0);
    m_shooterRightEncoder.SetAverageDepth(1.0);

    // rotation motor controller (SLOT 0) ENCODER GAINS
    m_rotationEncoder.SetPositionConversionFactor(kRotationsToDegrees);
    m_rotationEncoder.SetVelocityConversionFactor(kRotationsToDegrees / 60.0);

    m_rotationMotorController.SetFeedbackDevice(m_rotationEncoder);
    m_rotationMotorController.SetP(0.013, 0); // 0.00002
    m_rotationMotorController.SetI(0.0, 0); // 0
    m_rotationMotorController.SetD(0.27, 0); // 0.15
    m_rotationMotorController.SetFF(1.0 / 275.0, 0);

    m_rotationMotorController.SetIZone(4.0, 0);

    m_rotationMotorController.SetOutputRange(-1.0F, 1.0F, 0);
    m_rotationMotorController.SetSmartMotionAllowedClosedLoopError(2.0, 0);
    m_rotationMotor.EnableVoltageCompensation(12.0);

    m_rotationMotorController.SetSmartMotionMaxVelocity(175.0, 0);
    m_rotationMotorController.SetSmartMotionMaxAccel(750.0, 0);

    // rotation motor controller (SLOT 1) PIGEON GAINS
    m_rotationMotorController.SetP(0.04, 1); // 0.00002
    m_rotationMotorController.SetI(0.0002, 1); // 0
    m_rotationMotorController.SetD(0.05, 1); // 0.15
    m_rotationMotorController.SetFF(0.0, 1);

    m_rotationMotorController.SetIZone(4.0, 1);

    m_rotationMotorController.SetOutputRange(-1.0F, 1.0F, 1);
    m_rotationMotorController.SetSmartMotionAllowedClosedLoopError(1.0, 1);

    m_rotationMotorController.SetSmartMotionMaxVelocity(125.0, 1);
    m_rotationMotorController.SetSmartMotionMaxAccel(750.0, 1);

    // Burn after reading (2008)
    m_shooterMotorLeft.BurnFlash();
    m_shooterMotorRight.BurnFlash();
    m_rotationMotor.BurnFlash();
}

void intakeshooter::intakeActivate() {
    currentIntakeshooterState = intakeshooterStates::INTAKING;
}

void intakeshooter::intakeRetract() {
    currentIntakeshooterState = intakeshooterStates::IDLE;
}

void intakeshooter::spinup(float angle) { // provide manual angle control before vision is finished
    shootAngle = angle; // read shootAngle from angle (passed from command) when explicitly set

    if (allowSpinup) {
        currentIntakeshooterState = intakeshooterStates::SPINUP;
    }
}

void intakeshooter::scoreAmp() {
    currentIntakeshooterState = intakeshooterStates::AIMAMP;
}

void intakeshooter::rapidFire() {
    currentIntakeshooterState = intakeshooterStates::RAPIDFIRE;
}

void intakeshooter::fire() {
    if (m_rotationEncoder.GetPosition() >= 50) {
        currentIntakeshooterState = intakeshooterStates::FIRE;
    }
}

intakeshooterStates intakeshooter::getState() {
    return currentIntakeshooterState;
}

bool intakeshooter::shooterAtSpeed() {
    return m_shooterLeftEncoder.GetVelocity() > 4500.0 - kShooterRPMTolerance && m_shooterLeftEncoder.GetVelocity() < 4500.0 + kShooterRPMTolerance;
}

void intakeshooter::Periodic() {
    frc::SmartDashboard::PutNumber("Pigeon", -m_Pigeon.GetPitch().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Ab Angle", 246.138 - 180.0 + (-m_Pigeon.GetPitch().GetValueAsDouble()));

    // intakeshooter state machine
    switch (currentIntakeshooterState) {
        case intakeshooterStates::IDLE:
            currentLimitsConfigs.WithStatorCurrentLimit(40.0);
            m_intakeMotorLeft.GetConfigurator().Apply(currentLimitsConfigs);
            m_intakeMotorRight.GetConfigurator().Apply(currentLimitsConfigs);

            gravityFF = 0.0;
            m_rotationMotorController.SetReference(kIntakeResetAngle, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut); // 
            m_rotationEncoder.SetPosition(246.138 - 180.0 + (-m_Pigeon.GetPitch().GetValueAsDouble()));

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps)); // set the speed of the intake motor

            m_shooterMotorLeftController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity); // set the speed of the shooter motor (worse api b/c REV is cringe)
            m_shooterMotorRightController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity); // set speeds seperatly for spin while shooting
            // currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMR) ? intakeshooterStates::HOLDING : intakeshooterStates::IDLE; // if the canifier's limit forward input is tripped, switch to holding (for preloads)
            /* if (m_rotationEncoder.GetPosition() < 0){
                m_rotationEncoder.SetPosition(0);
            } */

            intakeState = "IDLE";
            break;
        case intakeshooterStates::INTAKING:
            gravityFF = 0.03 * sin(((0.0) - ((-1.0 * m_Pigeon.GetPitch().GetValueAsDouble()) * (M_PI/180.0))));

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(-45_tps)); //!!!!!!30
            m_rotationMotorController.SetReference(kIntakeIntakingAngle, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut); // !!!!!!118.0
            m_shooterMotorLeftController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
            m_shooterMotorRightController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);

            currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMR) ? intakeshooterStates::BACKOFF : intakeshooterStates::INTAKING; // if the canifier's limit forward input is tripped, switch to backoff

            intakeState = "INTAKING";
            break;
        case::intakeshooterStates::BACKOFF:
            allowSpinup = false;
            
            gravityFF = 0.1 * sin(((0.0) - ((-1.0 * m_Pigeon.GetPitch().GetValueAsDouble()) * (M_PI/180.0))));

            m_rotationMotorController.SetReference(kIntakeResetAngle, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkPIDController::SparkMaxPIDController::ArbFFUnits::kPercentOut); //retract intake when holding note

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(5_tps));

            currentIntakeshooterState = m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMR) ? intakeshooterStates::NOTEFORWARD : intakeshooterStates::BACKOFF;

            intakeState = "BACKOFF";

            m_controllerMain->SetRumble(frc2::CommandXboxController::RumbleType::kBothRumble, 1.0);
            m_controllerOperator->SetRumble(frc2::CommandXboxController::RumbleType::kBothRumble, 1.0);

            break;
        case intakeshooterStates::NOTEFORWARD:
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(-1_tps));

            currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMR) ? intakeshooterStates::HOLDING : intakeshooterStates::NOTEFORWARD;

            m_controllerMain->SetRumble(frc2::CommandXboxController::RumbleType::kBothRumble, 0.0);
            m_controllerOperator->SetRumble(frc2::CommandXboxController::RumbleType::kBothRumble, 0.0);

            intakeState = "NOTEFORWARD";
            break;
        case intakeshooterStates::HOLDING:
            allowSpinup = true;

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps)); // set the speed of the intake motor
            
            intakeState = "HOLDING";
            break;
        case intakeshooterStates::SPINUP:
            gravityFF = 0.05 * sin(((M_PI/3.0) - (m_rotationEncoder.GetPosition() * (M_PI/180.0))));

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps)); // set the speed of the intake motor

            m_shooterMotorLeftController.SetReference(4500, rev::CANSparkMax::ControlType::kVelocity); // set the speed of the shooter motors diferently so we have spin monkey
            m_shooterMotorRightController.SetReference(-4000, rev::CANSparkMax::ControlType::kVelocity);
            
            m_rotationMotorController.SetReference(shootAngle, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkPIDController::SparkMaxPIDController::ArbFFUnits::kPercentOut); // 110 angle for close shot speaker, 90 for far shot (originally), 93 from 12-14 feet (Tuesday), 99 from 3-4 feet away

            if (abs(m_rotationEncoder.GetPosition() - shootAngle) < 2.0) {
                currentIntakeshooterState = intakeshooterStates::SPINUPPIGEON;
            }

            intakeState = "SPINUP";
            break;
        case intakeshooterStates::SPINUPPIGEON:
            m_rotationEncoder.SetPosition(246.138 - 180.0 + (-m_Pigeon.GetPitch().GetValueAsDouble()));

            gravityFF = 0.05 * sin(((M_PI/3.0) - (246.138 - 180.0 + (-m_Pigeon.GetPitch().GetValueAsDouble()) * (M_PI/180.0))));
            
            m_rotationMotorController.SetReference(shootAngle, rev::CANSparkMax::ControlType::kPosition, 1, gravityFF, rev::SparkPIDController::SparkMaxPIDController::ArbFFUnits::kPercentOut); // 110 angle for close shot speaker, 90 for far shot (originally), 93 from 12-14 feet (Tuesday), 99 from 3-4 feet away

            intakeState = "SPINUPPIGEON";
            break;
        case intakeshooterStates::AMPBACK:
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(-1_tps));

            ampBackCount++;

            if (ampBackCount > 5) {
                currentIntakeshooterState = intakeshooterStates::AIMAMP;
                ampBackCount = 0;
            }

            intakeState = "AMPBACK";
            break;
        case intakeshooterStates::AIMAMP:
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps));

            gravityFF = 0.1 * sin(((0.0) - ((-1.0 * m_Pigeon.GetPitch().GetValueAsDouble()) * (M_PI/180.0))));

            m_rotationMotorController.SetReference(kIntakeAmpAngle, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkPIDController::SparkMaxPIDController::ArbFFUnits::kPercentOut);

            if (m_rotationEncoder.GetPosition() > 25) {
                currentIntakeshooterState = intakeshooterStates::SCOREAMP;
                counter = 0;
            }

            intakeState = "AIMAMP";
            break;
        case intakeshooterStates::SCOREAMP:
            if (counter < 50) {
                counter++;
            } else {
                currentLimitsConfigs.WithStatorCurrentLimit(80.0);
                
                m_intakeMotorLeft.GetConfigurator().Apply(currentLimitsConfigs);
                m_intakeMotorRight.GetConfigurator().Apply(currentLimitsConfigs);

                m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(20_tps));
                //currentIntakeshooterState = m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMR) ? intakeshooterStates::IDLE : intakeshooterStates::SCOREAMP; // if the canifier's limit backward input is tripped, switch to postfire

                intakeState = "SCOREAMP";
            }
            break;
        case intakeshooterStates::FIRE: //in the hole
            currentLimitsConfigs.WithStatorCurrentLimit(80.0);
            m_intakeMotorLeft.GetConfigurator().Apply(currentLimitsConfigs);
            m_intakeMotorRight.GetConfigurator().Apply(currentLimitsConfigs);

            m_rotationEncoder.SetPosition(246.138 - 180.0 + (-m_Pigeon.GetPitch().GetValueAsDouble()));

            //if (m_rotationEncoder.GetPosition() > shootAngle - kIntakeAngleTolerance && m_rotationEncoder.GetPosition() < shootAngle + kIntakeAngleTolerance){
                 m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(-50_tps)); // set the speed of the intake motor
            //}
            
            shooterClearCount++;
            currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMF) ? intakeshooterStates::POSTFIRE : intakeshooterStates::FIRE; // if the canifier's limit backward input is tripped, switch to postfire
            //currentIntakeshooterState = shooterClearCount > 4 ? intakeshooterStates::POSTFIRE : currentIntakeshooterState;
            intakeState = "FIRE";
            break;
        case intakeshooterStates::RAPIDFIRE:
            if (m_rotationEncoder.GetPosition() > shootAngle - kIntakeAngleTolerance && m_rotationEncoder.GetPosition() < shootAngle + kIntakeAngleTolerance){
                 m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(-50_tps)); // set the speed of the intake motor
            }
            currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMF) ? intakeshooterStates::RAPIDPOSTFIRE : intakeshooterStates::RAPIDFIRE;
            intakeState = "RAPIDFIRE";
            break;
        case::intakeshooterStates::POSTFIRE:
            shooterClearCount = 0;
            currentIntakeshooterState = m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMF) ? intakeshooterStates::IDLE : intakeshooterStates::POSTFIRE;

            intakeState = "POSTFIRE";
            break;
        case::intakeshooterStates::RAPIDPOSTFIRE:
            currentIntakeshooterState = m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMF) ? intakeshooterStates::INTAKING : intakeshooterStates::RAPIDPOSTFIRE;

            intakeState = "RAPIDPOSTFIRE";
            break;
        case::intakeshooterStates::ZEROING:
            gravityFF = 0.0;

            m_rotationMotorController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkPIDController::SparkMaxPIDController::ArbFFUnits::kPercentOut); // set the angle 
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps));

            m_shooterMotorLeftController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
            m_shooterMotorRightController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);

            if (m_rotationEncoder.GetPosition() < 1.0) {
                m_rotationEncoder.SetPosition(0.0);
                currentIntakeshooterState = intakeshooterStates::IDLE;
            }

            intakeState = "ZEROING";
            break;
    }

    frc::SmartDashboard::PutNumber("Intake Rot", m_rotationEncoder.GetPosition());
    frc::SmartDashboard::PutString("Intake/Shooter State: ", intakeState);
    frc::SmartDashboard::PutNumber("Rot Output", m_rotationMotor.GetAppliedOutput());

    frc::SmartDashboard::PutNumber("Shtr Motor Output", m_shooterMotorLeft.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Shtr Out Curr", m_shooterMotorLeft.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shtr RPM", m_shooterLeftEncoder.GetVelocity());
    frc::SmartDashboard::PutBoolean("Shooter At Speed", shooterAtSpeed());
    // frc::SmartDashboard::PutNumber("GravFF", gravityFF);
    frc::SmartDashboard::PutNumber("Intake RPM", m_intakeMotorLeft.GetVelocity().GetValueAsDouble());
}

