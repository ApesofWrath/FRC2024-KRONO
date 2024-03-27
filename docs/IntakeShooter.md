The IntakeShooter is a generic subsystem, driven by various motors. It is declared as a class.
## Initialization
Initialized with 2 Krakens and 3 Neos. This is due to the acquisition of Krakens partway through construction. Also initialize the beam break.
### Krakens
The Kraken motors are `m_intakeMotorLeft` and `m_intakeMotorRight`, and as such those motors benefit from the superior `phoenix6` API. They are initialized in the header as `ctre::phoenix6::hardware::TalonFX`. `m_intakeMotorRight` is set to follow `kIntakeMotorLeft`, the ID of the left motor.

They require `ctre::phoenix6::configs::MotorOutputConfigs`, `ctre::phoenix6::configs::CurrentLimitsConfigs`, and `ctre::phoenix6::configs::Slot0Configs` objects for configuration. The configuration objects applied using the `GetConfigurator().Apply()` function.
* Change `motorOutputConfigs` so that it will coast when in neutral.
* Change `currentLimitsConfigs` to enable a limit on the current directed to the motors and set that limit.
* Change `slotZeroConfigs` to set the P of the motor (as in PID)

`m_velocityIntake` represents the rotational speed of the intake motors in turns per second.
* `WithSlot‎` changes the preset for the PID value, which we don't actually use. This is also why the PID is changed using slotZeroConfigs.
* `WithEnableFOC‎` toggles FOC, an optimization feature which we do not use.
### Neos
The Neo motors are `m_shooterMotorLeft`, `m_shooterMotorRight`, and `m_rotationMotor`, the shooter wheels and mechanism pivot motors. Initialized as `rev::CANSparkMax`. Configuration requires lots of redundant calls which cannot be optimized.
* `RestoreFactoryDefaults()` does what it says on the tin.
* `SetIdleMode()` makes motor coast or brake in neutral, coast for the shooters and break for rotation.
* `SetSmartCurrentLimit‎()` sets the limit of the current directed to that motor.
* `BurnFlash()` 

`m_shooterLeftEncoder` and `m_shooterRightEncoder` are the `rev::SparkMaxRelativeEncoder` for their matching shooter motors, created using the `GetEncoder()` function to get the `kHallEffect` encoder at 42 counts per revolution. 
* `SetMeasurementPeriod()` set the period (change in time) used to calculate the velocity from the encoder.
* `SetAverageDepth‎()` set number of samples to use for the averaging in the velocity calculations.

The `m_shooterMotorLeftController`, `m_shooterMotorRightController`, and `m_rotationMotorController` are the PID controllers of their corresponding motors, initialized as `rev::SparkPIDController` using their motor's `GetPIDController()` functions.
* `SetP(0)`, `SetI(0)`, and `SetD(0)` set the PID values for the motor.
* `SetFF()` set the PID controller's feed-forward gain
* `SetOutputRange‎()` sets the maximum and minimum speed that the motor is allowed to go.

`m_rotationMotorController` (the pivot motor's controller) uses Alternate Encoder Mode, and as such requires `m_rotationEncoder` (a `rev::SparkMaxAlternateEncoder`). The physical encoder is one of REV's through bore encoders, which has a counts per revolution of 8192.
* `SetFeedbackDevice()` sets the controller's feedback device, here to the encoder
* `SetPositionConversionFactor()` sets the conversion factor for position of the encoder by multiplying the raw output, here by 360 to turn from rotations to degrees.
* `SetVelocityConversionFactor‎()` is similar, using 360/60 to go from rotations to degrees per second.
* `SetSmartMotionAllowedClosedLoopError‎()` TODO, cannot find docs
* `SetSmartMotionMaxVelocity‎()` TODO, cannot find docs
* `SetSmartMotionMaxAccel()` TODO, cannot find docs

## State
### Enum types
`intakeshooterStates`: store the states of the mechanism
* `IDLE`: Default state, wheels stop and gravityFF is zeroed. Move to holding if beam break tripped.
* `INTAKING`: Enter on intake button, intake angles down (compensating for gravity) & spins wheels. Move to backoff if beam break tripped.
* `BACKOFF`: Run the intake wheels backwards a little bit to position the note correctly, then switch to holding.
* `HOLDING`: Angle the shooter correctly (compensating for gravity)
* `SPINUP`: enter on rev button press, spin wheels at different speeds to achieve spin, angle shooter to target the target (compensating for gravity and using shootAngle, discussed below)
* `FIRE`: Enter on fire button, feed note to shooter wheels, go to postfire when beam break tripped
* `POSTFIRE`: Go to zeroing when beam break tripped
* `ZEROING`: Zero gravityFF, shooter motor speeds, intake motor speeds, and mechanism angle. Go to idle when this is done.

### State setters
`intakeActivate()`, `spinup()`, and `fire()` set the value of `currentIntakeshooterState` to the appropriate values. These functions are called by commands, the code of which is stored in `src/main/(cpp|include)/commands/shooterintake`. The commands are called on button presses declared in `RobotContainer.cpp` using the `RobotContainer::ConfigureButtonBindings()` function. Spinup is more complicated, as it uses an overloaded command constructor to ether aim from a command argument or aim based on limelight.

## Periodic
`intakeshooter::Periodic()` consists of the state machine itself, a `switch` statement with `case`s for each possible value of `intakeshooterStates`. The motors are controlled by the code here using the following functions.
* `SetControl()`: control the velocity of the Kraken motors
* `SetReference‎()`: control the `rev::CANSparkMax::ControlType::kVelocity` and `rev::CANSparkMax::ControlType::kPosition` of Neo motors

`m_BeambreakCanifier` is a `ctre::phoenix::CANifier` which we use to access the status of two beam breaks, detecting the note's entrance and exit from the holding area. We do this by running the `GetGeneralInput()` function using class objects `ctre::phoenix::CANifier::LIMF` and `ctre::phoenix::CANifier::LIMR`, representing the forward limit and reverse limit ports. These ports are plugged in to the shooter and intake beam breaks respectively.