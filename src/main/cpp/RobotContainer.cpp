// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

using namespace generalConstants;
RobotContainer::RobotContainer() {

	// Initialize all of your commands and subsystems here
	pathplanner::NamedCommands::registerCommand("spinup", std::move(spinup(&m_intakeshooter, 115.5).ToPtr()));
	pathplanner::NamedCommands::registerCommand("fire", std::move(fire(&m_intakeshooter).ToPtr()));
	pathplanner::NamedCommands::registerCommand("intakeActivate", std::move(intakeActivate(&m_intakeshooter).ToPtr()));
	pathplanner::NamedCommands::registerCommand("intakeRetract", std::move(intakeRetract(&m_intakeshooter).ToPtr()));
	pathplanner::NamedCommands::registerCommand("rapidFire", std::move(rapidFire(&m_intakeshooter).ToPtr()));
	// Configure the button bindings
	ConfigureButtonBindings();

	// $ CONTROLLER INPUTS FOR SWERVE DRIVE BELOW
	m_drivetrain.SetDefaultCommand(Drive(
		&m_drivetrain,
		[this] { return (MathFunctions::joystickCurve((m_controllerMain.GetLeftX()), controllerConstants::kControllerCurve)); },
		[this] { return (MathFunctions::joystickCurve((m_controllerMain.GetLeftY()), controllerConstants::kControllerCurve)); },
		[this] { return (m_controllerMain.GetRawAxis(4)); }
	));

	m_chooser.SetDefaultOption("DoNothing", "DoNothing");
	m_chooser.AddOption("2NoteCenter", "2NoteCenter");
	m_chooser.AddOption("2NoteAmpSide", "2NoteAmpSide");
	m_chooser.AddOption("3NoteAmpSide", "3NoteAmpSide");
	m_chooser.AddOption("3NoteCenter", "3NoteCenter");
	m_chooser.AddOption("4Note", "4Note");
	m_chooser.AddOption("Backup", "Backup");
	m_chooser.AddOption("Preload", "Preload");
	m_chooser.AddOption("PreloadBackupCenter", "PreloadBackupCenter");
	m_chooser.AddOption("New Auto", "New Auto");

	frc::SmartDashboard::PutData(&m_chooser);
		
	// Start Timer
	timer.Start();
}

// All the button commands are set in this function
void RobotContainer::ConfigureButtonBindings() {
	// regular command stuff so far↴  command declaration↴ lambda with the code↴
	// m_controller.Button().Trigger( m_subsystem.RunOnce( [this]{ commandCode(); } ) )

	// Zeroing for swervedrive command
	m_controllerMain.Start().OnTrue(m_drivetrain.RunOnce([this]{m_drivetrain.resetGyro();}));

	// Slow button for swerve (whenever left OR right bumper is held down), slows swerve to slow value
	m_controllerMain.RightBumper().OnTrue(m_drivetrain.RunOnce([this]{m_drivetrain.slowDown();}))
		.OnFalse(m_drivetrain.RunOnce([this]{m_drivetrain.normalSpeed();}));
	m_controllerMain.LeftBumper().OnTrue(m_drivetrain.RunOnce([this]{m_drivetrain.slowDown();}))
		.OnFalse(m_drivetrain.RunOnce([this]{m_drivetrain.normalSpeed();}));

	// Align
	m_controllerMain.B().WhileTrue(m_drivetrain.RunOnce([this]{
		double heading_error = m_vision.getHeadingError();
		units::angular_velocity::radians_per_second_t heading_error_radians{heading_error};
		m_drivetrain.SwerveDrive(0.0_mps, 0.0_mps, heading_error_radians, false);
	}));

	// ShooterIntake buttons
	m_controllerOperator.LeftBumper().OnTrue(m_intakeshooter.RunOnce([this]{m_intakeshooter.intakeActivate();})); // kA
	m_controllerOperator.B().OnTrue(AutoAngle(&m_intakeshooter, &m_vision).ToPtr()); // spinup for far speaker shot (7 feet from speaker)                       <
	m_controllerOperator.X().OnTrue(spinup(&m_intakeshooter, intakeConstants::kIntakeSpeakerAngle).ToPtr()); // spinup for near speaker shot (right at speaker) < TODO: these three are monsters
	m_controllerOperator.RightBumper().OnTrue(fire(&m_intakeshooter).ToPtr()); //                                                                               <
	m_controllerOperator.A().OnTrue(m_intakeshooter.RunOnce([this]{m_intakeshooter.intakeRetract();})); //leftbumper
	m_controllerOperator.Y().OnTrue(m_intakeshooter.RunOnce([this]{m_intakeshooter.scoreAmp();}));
	
	// Climber Buttons
	m_controllerOperator.LeftStick().OnTrue(m_climber.RunOnce([this]{m_climber.climberExtend();}));
	m_controllerOperator.RightStick().OnTrue(m_climber.RunOnce([this]{m_climber.climberRetract();}));

	// Climber Zero Maintinence Buttons
	m_controllerAlt.LeftBumper().OnTrue(m_climber.RunOnce([this]{m_climber.leftClimbToggle();}));
	m_controllerAlt.RightBumper().OnTrue(m_climber.RunOnce([this]{m_climber.rightClimbToggle();}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	using namespace pathplanner;
	return m_chooser.GetSelected() == "DoNothing" ? frc2::WaitCommand(15_s).ToPtr() : PathPlannerAuto(m_chooser.GetSelected()).ToPtr();
}