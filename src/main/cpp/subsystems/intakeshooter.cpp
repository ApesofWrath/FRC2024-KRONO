#pragma once
#include "subsystems/intakeshooter.h"
using namespace shooterConstants;
using namespace intakeConstants;
using namespace generalConstants;

intakeshooter::intakeshooter()
: m_intakeMotorLeft(kIntakeMotorLeft),
m_intakeMotorRight(kIntakeMotorRight),
m_shooterMotorLeft(kMotorShooterLeft, rev::CANSparkMax::MotorType::kBrushless),
m_shooterMotorRight(kMotorShooterRight, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(kIntakeRotationMotor, rev::CANSparkMax::MotorType::kBrushless),
m_BeambreakCanifier(kBeambreakCanifier)
{

    //Kraken Settings
    m_intakeMotorRight.SetControl(ctre::phoenix6::controls::Follower(kIntakeMotorLeft, false));
    motorOutputConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

    currentLimitsConfigs.WithStatorCurrentLimitEnable(true);
    currentLimitsConfigs.WithStatorCurrentLimit(25.0);

    // Kraken PID Values
    slotZeroConfigs.WithKP(0.004);
    slotZeroConfigs.WithKI(0.0);
    slotZeroConfigs.WithKD(0.0);
    slotZeroConfigs.WithKV(12.0 / 100.0);

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
    
    // m_rotationMotor.RestoreFactoryDefaults();

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

    // rotation motor controller
    m_rotationEncoder.SetPositionConversionFactor(kRotationsToDegrees);
    m_rotationEncoder.SetVelocityConversionFactor(kRotationsToDegrees / 60.0);

    m_rotationMotorController.SetFeedbackDevice(m_rotationEncoder);
    m_rotationMotorController.SetP(0.00004);
    m_rotationMotorController.SetI(0);
    m_rotationMotorController.SetD(0);
    m_rotationMotorController.SetFF(1.0 / 275.0);
    m_rotationMotorController.SetOutputRange(-1.0F, 1.0F);
    m_rotationMotorController.SetSmartMotionAllowedClosedLoopError(2.0);

    m_rotationMotorController.SetSmartMotionMaxVelocity(125.0);
    m_rotationMotorController.SetSmartMotionMaxAccel(750.0);

    m_rotationMotor.BurnFlash();
}

void intakeshooter::intakeActivate() {
    currentIntakeshooterState = intakeshooterStates::INTAKING;

    // currentTestState = testStates::down;
}
void intakeshooter::spinup() {
    currentIntakeshooterState = intakeshooterStates::SPINUP;

    // currentTestState = testStates::up;
}
void intakeshooter::fireSPEAKER() {
    currentShootTarget = shootTarget::SPEAKER;
    currentIntakeshooterState = intakeshooterStates::FIRE;

    //currentTestState = testStates::down;
}
void intakeshooter::fireAMP() {
    currentShootTarget = shootTarget::AMP;
    currentIntakeshooterState = intakeshooterStates::FIRE;

    //currentTestState = testStates::up;
}

void intakeshooter::Periodic() {

    double gravityFF = 0.0;

    // intakeshooter state machine
    switch (currentIntakeshooterState) {
        case intakeshooterStates::IDLE:
            gravityFF = 0.0;

            // m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(2.5_tps)); // set the speed of the intake motor
            // m_shooterMotorLeftController.SetReference(0.25, rev::CANSparkMax::ControlType::kDutyCycle); // set the speed of the shooter motor (worse api b/c REV is cringe)
            m_rotationMotorController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut); // set the angle 
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps));

            m_shooterMotorLeftController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
            m_shooterMotorRightController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
            currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMR) ? intakeshooterStates::HOLDING : intakeshooterStates::IDLE; // if the canifier's limit forward input is tripped, switch to holding

            intakeState = "IDLE";
            break;
        case intakeshooterStates::INTAKING:
            gravityFF = 0.03 * sin(((M_PI/3.0) - (m_rotationEncoder.GetPosition() * (M_PI/180.0))));

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(7_tps));
            m_rotationMotorController.SetReference(118, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut);
            currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMR) ? intakeshooterStates::BACKOFF : intakeshooterStates::INTAKING; // if the canifier's limit forward input is tripped, switch to holding

            intakeState = "INTAKING";
            break;
        case::intakeshooterStates::BACKOFF:
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(-1_tps));
            backoffCount++;

            if (backoffCount>8) {
                currentIntakeshooterState = intakeshooterStates::HOLDING;
                backoffCount=0;
            }
            break;
        case intakeshooterStates::HOLDING:
            gravityFF = 0.1 * sin(((M_PI/3.0) - (m_rotationEncoder.GetPosition() * (M_PI/180.0))));

            // m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0.0_tps)); // set the speed of the intake motor
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps));
            
            m_rotationMotorController.SetReference(118, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut);

            intakeState = "HOLDING";
            break;
        case intakeshooterStates::SPINUP:
            gravityFF = 0.1 * sin(((M_PI/3.0) - (m_rotationEncoder.GetPosition() * (M_PI/180.0))));
            // m_shooterMotorLeftController.SetReference(1.0, rev::CANSparkMax::ControlType::kDutyCycle); // set the speed of the shooter motor (TODO: trial and error with rpm)

            m_shooterMotorLeftController.SetReference(5000, rev::CANSparkMax::ControlType::kVelocity);
            m_shooterMotorRightController.SetReference(-5400, rev::CANSparkMax::ControlType::kVelocity);
            
            m_rotationMotorController.SetReference(90, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut); // 110 angle for close shot speaker

            intakeState = "SPINUP";
            break;
        case intakeshooterStates::FIRE: //in the hole
            //m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(1.0_tps)); // set the speed of the intake motor (TODO: tune speed)
            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(50_tps));
            // m_rotationMotorController.SetReference(30, rev::CANSparkMax::ControlType::kPosition); // set the angle (TODO: tune angle)
            currentIntakeshooterState = !m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMF) ? intakeshooterStates::POSTFIRE : intakeshooterStates::FIRE; // if the canifier's limit backward input is tripped, switch to idle

            intakeState = "FIRE";
            break;
        case::intakeshooterStates::POSTFIRE:
            currentIntakeshooterState = m_BeambreakCanifier.GetGeneralInput(ctre::phoenix::CANifier::LIMF) ? intakeshooterStates::IDLE : intakeshooterStates::POSTFIRE;
        
            break;
    }

    switch(currentTestState) {
        case testStates::up:
            // gravityFF = 3 * sin(((M_PI/3.0) - (m_rotationEncoder.GetPosition() * (M_PI/180.0))));
            gravityFF = 0;
            //m_rotationMotorController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF);
            // m_shooterMotorLeftController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(0_tps));
            rotState = "off";
            break;
        
        case testStates::down:
            gravityFF = 0.07 * sin(((M_PI/3.0) - (m_rotationEncoder.GetPosition() * (M_PI/180.0))));
            //m_rotationMotorController.SetReference(118, rev::CANSparkMax::ControlType::kSmartMotion, 0, gravityFF, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut);
            /* m_shooterMotorLeftController.SetReference(2500, rev::CANSparkMax::ControlType::kVelocity);
            m_shooterMotorRightController.SetReference(-2500, rev::CANSparkMax::ControlType::kVelocity); */

            m_intakeMotorLeft.SetControl(m_velocityIntake.WithVelocity(250_tps));
            rotState = "on";
            break;

        case testStates::idle:

            break;
    }

    frc::SmartDashboard::PutNumber("Intake Rot", m_rotationEncoder.GetPosition());
    frc::SmartDashboard::PutString("Intake/Shooter State: ", intakeState);
    frc::SmartDashboard::PutString("Rot State", rotState);

    frc::SmartDashboard::PutNumber("Shtr Motor Output", m_shooterMotorLeft.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Shtr Out Curr", m_shooterMotorLeft.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shtr RPM", m_shooterLeftEncoder.GetVelocity());
    // frc::SmartDashboard::PutNumber("GravFF", gravityFF);
}