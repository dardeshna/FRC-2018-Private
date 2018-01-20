package com.palyrobotics.frc2018.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.util.ClimberSignal;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.trajectory.Kinematics;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;
import com.palyrobotics.frc2018.util.trajectory.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;
import java.util.logging.Level;

/**
 * Should only be used in robot package.
 */
class HardwareUpdater {

	// Subsystem references
	private Drive mDrive;
	private Climber mClimber;
	private Elevator mElevator;

	/**
	 * Hardware Updater for Forseti
	 */
	HardwareUpdater(Drive drive, Climber climber, Elevator elevator) throws Exception {
		this.mDrive = drive;
		this.mClimber = climber;
		this.mElevator = elevator;
	}

	/**
	 * Initialize all hardware
	 */
	void initHardware() {
		Logger.getInstance().logRobotThread(Level.INFO,"Init hardware");
		configureTalons();
		PigeonIMU gyro = HardwareAdapter.getInstance().getDrivetrain().gyro;
		gyro.setYaw(0, 0);
		gyro.setFusedHeading(0, 0);
	}

	void disableTalons() {
		Logger.getInstance().logRobotThread(Level.INFO,"Disabling talons");

		//Drivetrain disables
		HardwareAdapter.getInstance().getDrivetrain().leftMasterTalon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().leftSlave1Talon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().rightSlave1Talon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().leftSlave2Talon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().rightSlave2Talon.set(ControlMode.Disabled, 0);

		//Climber disables
		HardwareAdapter.getInstance().getClimber().leftVictor.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getClimber().rightVictor.set(ControlMode.Disabled, 0);

		//Elevator disables
		HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getElevator().elevatorSlaveTalon.set(ControlMode.Disabled, 0);
	}

	void configureTalons() {
		configureDriveTalons();
		configureClimberTalons();
		configureElevatorTalons();
	}

	void configureDriveTalons() {
		WPI_TalonSRX leftMasterTalon = HardwareAdapter.getInstance().getDrivetrain().leftMasterTalon;
		WPI_TalonSRX leftSlave1Talon = HardwareAdapter.getInstance().getDrivetrain().leftSlave1Talon;
		WPI_TalonSRX leftSlave2Talon = HardwareAdapter.getInstance().getDrivetrain().leftSlave2Talon;
		WPI_TalonSRX rightMasterTalon = HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon;
		WPI_TalonSRX rightSlave1Talon = HardwareAdapter.getInstance().getDrivetrain().rightSlave1Talon;
		WPI_TalonSRX rightSlave2Talon = HardwareAdapter.getInstance().getDrivetrain().rightSlave2Talon;

		// Enable all talons' brake mode and disables forward and reverse soft
		leftMasterTalon.setNeutralMode(NeutralMode.Brake);
		leftSlave1Talon.setNeutralMode(NeutralMode.Brake);
		if (leftSlave2Talon != null) leftSlave2Talon.setNeutralMode(NeutralMode.Brake);
		rightMasterTalon.setNeutralMode(NeutralMode.Brake);
		rightSlave1Talon.setNeutralMode(NeutralMode.Brake);
		if (rightSlave2Talon != null) rightSlave2Talon.setNeutralMode(NeutralMode.Brake);

		leftMasterTalon.enableVoltageCompensation(true);
		leftSlave1Talon.enableVoltageCompensation(true);
		leftSlave2Talon.enableVoltageCompensation(true);

		rightMasterTalon.enableVoltageCompensation(true);
		rightSlave1Talon.enableVoltageCompensation(true);
		rightSlave2Talon.enableVoltageCompensation(true);

		leftMasterTalon.configForwardSoftLimitEnable(false, 0);
		leftMasterTalon.configReverseSoftLimitEnable(false, 0);
		leftSlave1Talon.configForwardSoftLimitEnable(false, 0);
		leftSlave1Talon.configReverseSoftLimitEnable(false, 0);

		if (rightSlave2Talon != null) {rightSlave2Talon.configForwardSoftLimitEnable(false, 0); rightSlave2Talon.configReverseSoftLimitEnable(false, 0);}

		rightMasterTalon.configForwardSoftLimitEnable(false, 0);
		rightMasterTalon.configReverseSoftLimitEnable(false, 0);
		rightSlave1Talon.configForwardSoftLimitEnable(false, 0);
		rightSlave1Talon.configReverseSoftLimitEnable(false, 0);

		if (rightSlave2Talon != null) {rightSlave2Talon.configForwardSoftLimitEnable(false, 0); rightSlave2Talon.configReverseSoftLimitEnable(false, 0);}

		// Allow max voltage for closed loop control
		leftMasterTalon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		leftMasterTalon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
		leftSlave1Talon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		leftSlave1Talon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);

		if (leftSlave2Talon != null) {
			leftSlave2Talon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
			leftSlave2Talon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
		}

		rightMasterTalon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		rightMasterTalon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
		rightSlave1Talon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		rightSlave1Talon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);

		if (rightSlave2Talon != null) {
			rightSlave2Talon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
			rightSlave2Talon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
		}

		// Configure master talon feedback devices
		leftMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

		leftMasterTalon.setSensorPhase(false);
		rightMasterTalon.setSensorPhase(false);

		leftMasterTalon.setStatusFramePeriod(0, 5, 0);
		rightMasterTalon.setStatusFramePeriod(0, 5, 0);

		leftMasterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 0);
		rightMasterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 0);

		leftMasterTalon.configVelocityMeasurementWindow(16, 0);
		rightMasterTalon.configVelocityMeasurementWindow(16, 0);

		// Zero encoders
		leftMasterTalon.setSelectedSensorPosition(0, 0, 0);
		rightMasterTalon.setSelectedSensorPosition(0, 0, 0);

		// Reverse right side
		rightMasterTalon.setInverted(true);
		rightSlave1Talon.setInverted(true);
		rightSlave2Talon.setInverted(true);

		// Set slave talons to follower mode
		leftSlave1Talon.set(ControlMode.Follower, leftMasterTalon.getDeviceID());
		if (leftSlave2Talon != null) {
			leftSlave2Talon.set(ControlMode.Follower, leftMasterTalon.getDeviceID());
		}
		rightSlave1Talon.set(ControlMode.Follower, rightMasterTalon.getDeviceID());
		if (rightSlave2Talon != null) {
			rightSlave2Talon.set(ControlMode.Follower, rightMasterTalon.getDeviceID());
		}
	}
	
	void configureElevatorTalons() {
		WPI_TalonSRX masterTalon = HardwareAdapter.getInstance().getElevator().elevatorMasterTalon;
		WPI_TalonSRX slaveTalon = HardwareAdapter.getInstance().getElevator().elevatorSlaveTalon;
		slaveTalon.set(ControlMode.Follower, masterTalon.getDeviceID());

		masterTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 100);
		masterTalon.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, Constants.kForsetiElevatorSlaveTalonID, 100);

		masterTalon.enableVoltageCompensation(true);
		slaveTalon.enableVoltageCompensation(true);

		masterTalon.configVoltageCompSaturation(14, 0);
		slaveTalon.configVoltageCompSaturation(14, 0);

		//TODO: which way is up and how does it work w/ inverted
		masterTalon.configNominalOutputForward(Constants.kNominalUpwardsOutput, 0);
		masterTalon.configNominalOutputReverse(0, 0);
		slaveTalon.configNominalOutputForward(Constants.kNominalUpwardsOutput, 0);
		slaveTalon.configNominalOutputReverse(0, 0);

		masterTalon.configPeakOutputForward(Constants.kElevatorMaxClosedLoopOutput, 0);
		masterTalon.configPeakOutputReverse(-Constants.kElevatorMaxClosedLoopOutput, 0);
		slaveTalon.configPeakOutputForward(Constants.kElevatorMaxClosedLoopOutput, 0);
		slaveTalon.configPeakOutputReverse(-Constants.kElevatorMaxClosedLoopOutput, 0);

		masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		//TODO: in phase or not?
		masterTalon.setSensorPhase(true);

		// Zero encoders
		masterTalon.setSelectedSensorPosition(0, 0, 0);

		// Reverse right side
		slaveTalon.setInverted(true);
	}

	void configureClimberTalons() {
		WPI_VictorSPX climberLeft = HardwareAdapter.getInstance().getClimber().leftVictor;
		WPI_VictorSPX climberRight = HardwareAdapter.getInstance().getClimber().rightVictor;

		climberLeft.enableVoltageCompensation(true);
		climberRight.enableVoltageCompensation(true);

		climberLeft.setNeutralMode(NeutralMode.Brake);
		climberRight.setNeutralMode(NeutralMode.Brake);

		climberLeft.configForwardSoftLimitEnable(false, 0);
		climberLeft.configReverseSoftLimitEnable(false, 0);
		climberRight.configForwardSoftLimitEnable(false, 0);
		climberRight.configReverseSoftLimitEnable(false, 0);
	}

	/**
	 * Updates all the sensor data taken from the hardware
	 */
	void updateState(RobotState robotState) {
		WPI_TalonSRX leftMasterTalon = HardwareAdapter.getInstance().getDrivetrain().leftMasterTalon;
		WPI_TalonSRX rightMasterTalon = HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon;

		robotState.leftControlMode = leftMasterTalon.getControlMode();
		robotState.rightControlMode = rightMasterTalon.getControlMode();

		robotState.leftStickInput.update(HardwareAdapter.getInstance().getJoysticks().driveStick);
		robotState.rightStickInput.update(HardwareAdapter.getInstance().getJoysticks().turnStick);
		robotState.operatorStickInput.update(HardwareAdapter.getInstance().getJoysticks().operatorStick);
		robotState.elevatorStickInput.update(HardwareAdapter.getInstance().getJoysticks().elevatorStick);
		
        switch(robotState.leftControlMode) {
            //Fall through
            case Position:
            case Velocity:
            case MotionProfileArc:
            case MotionProfile:
            case MotionMagicArc:
            case MotionMagic:
                robotState.leftSetpoint = leftMasterTalon.getClosedLoopTarget(0);
                break;
            case Current:
                robotState.leftSetpoint = leftMasterTalon.getOutputCurrent();
                break;
            //Fall through
            case Follower:
            case PercentOutput:
                robotState.leftSetpoint = leftMasterTalon.getMotorOutputPercent();
                break;
        }

        switch(robotState.rightControlMode) {
            //Fall through
            case Position:
            case Velocity:
            case MotionProfileArc:
            case MotionProfile:
            case MotionMagicArc:
            case MotionMagic:
                robotState.rightSetpoint = rightMasterTalon.getClosedLoopTarget(0);
                break;
            case Current:
                robotState.rightSetpoint = rightMasterTalon.getOutputCurrent();
                break;
            //Fall through
            case Follower:
            case PercentOutput:
                robotState.rightSetpoint = rightMasterTalon.getMotorOutputPercent();
                break;
        }
		
		PigeonIMU gyro = HardwareAdapter.getInstance().getDrivetrain().gyro;
		if (gyro != null) {
			robotState.drivePose.heading = gyro.getFusedHeading();
			robotState.drivePose.headingVelocity = (robotState.drivePose.heading - robotState.drivePose.lastHeading)/Constants.kNormalLoopsDt;
			robotState.drivePose.lastHeading = gyro.getFusedHeading();
		} else {
			robotState.drivePose.heading = -0;
			robotState.drivePose.headingVelocity = -0;
		}

		robotState.drivePose.lastLeftEnc = robotState.drivePose.leftEnc;
		robotState.drivePose.leftEnc = leftMasterTalon.getSelectedSensorPosition(0);
		robotState.drivePose.leftEncVelocity = leftMasterTalon.getSelectedSensorVelocity(0);
		robotState.drivePose.lastRightEnc = robotState.drivePose.rightEnc;
		robotState.drivePose.rightEnc = rightMasterTalon.getSelectedSensorPosition(0);
		robotState.drivePose.rightEncVelocity = rightMasterTalon.getSelectedSensorVelocity(0);

		if (leftMasterTalon.getControlMode().equals(ControlMode.MotionMagic)) {
			robotState.drivePose.leftMotionMagicPos = Optional.of(leftMasterTalon.getActiveTrajectoryPosition());
			robotState.drivePose.leftMotionMagicVel = Optional.of(leftMasterTalon.getActiveTrajectoryVelocity());
		}
		else {
			robotState.drivePose.leftMotionMagicPos = Optional.empty();
			robotState.drivePose.leftMotionMagicVel = Optional.empty();
		}

		if (rightMasterTalon.getControlMode().equals(ControlMode.MotionMagic)) {
			robotState.drivePose.rightMotionMagicPos = Optional.of(rightMasterTalon.getActiveTrajectoryPosition());
			robotState.drivePose.rightMotionMagicVel = Optional.of(rightMasterTalon.getActiveTrajectoryVelocity());
		}
		else {
			robotState.drivePose.rightMotionMagicPos = Optional.empty();
			robotState.drivePose.rightMotionMagicVel = Optional.empty();
		}

		robotState.drivePose.leftError = Optional.of(leftMasterTalon.getClosedLoopError(0));
        robotState.drivePose.rightError = Optional.of(rightMasterTalon.getClosedLoopError(0));

        double time = Timer.getFPGATimestamp();
        double left_distance = robotState.drivePose.leftEnc / Constants.kDriveTicksPerInch;
        double right_distance = robotState.drivePose.rightEnc / Constants.kDriveTicksPerInch;

//        Rotation2d gyro_angle = Rotation2d.fromRadians((right_distance - left_distance) * Constants.kTrackScrubFactor / Constants.kTrackEffectiveDiameter);
        Rotation2d gyro_angle = Rotation2d.fromDegrees(robotState.drivePose.heading);
		RigidTransform2d odometry = robotState.generateOdometryFromSensors(
                left_distance - robotState.drivePose.lastLeftEnc / Constants.kDriveTicksPerInch,
				right_distance - robotState.drivePose.lastRightEnc / Constants.kDriveTicksPerInch, gyro_angle);
        RigidTransform2d.Delta velocity = Kinematics.forwardKinematics(
        		robotState.drivePose.leftEncVelocity,
        		robotState.drivePose.rightEncVelocity
        );

        robotState.addObservations(time, odometry, velocity);
        
        // Update elevator sensors
		robotState.elevatorPosition = HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.getSelectedSensorPosition(0);
		robotState.elevatorVelocity = HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.getSelectedSensorVelocity(0);
        robotState.elevatorBottomHFX = HardwareAdapter.getInstance().getElevator().bottomHFX.get();
        robotState.elevatorTopHFX = HardwareAdapter.getInstance().getElevator().topHFX.get();
	}

	/**
	 * Updates the hardware to run with output values of subsystems
	 */
	void updateHardware() {
		updateDrivetrain();
		updateClimber();
		updateElevator();
	}

	/**
	 * Updates the drivetrain
	 * Uses TalonSRXOutput and can run off-board control loops through SRX
	 */
	private void updateDrivetrain() {
		updateTalonSRX(HardwareAdapter.getInstance().getDrivetrain().leftMasterTalon, mDrive.getDriveSignal().leftMotor);
		updateTalonSRX(HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon, mDrive.getDriveSignal().rightMotor);
	}
	
	/**
	 * Updates the elevator
	 */
	private void updateElevator() {
		updateTalonSRX(HardwareAdapter.getInstance().getElevator().elevatorMasterTalon, mElevator.getOutput());
	}

	private void updateClimber() {
		ClimberSignal signal = mClimber.getSignal();
		HardwareAdapter.getInstance().getClimber().leftVictor.set(signal.leftVelocity);
		HardwareAdapter.getInstance().getClimber().rightVictor.set(signal.rightVelocity);
		HardwareAdapter.getInstance().getClimber().leftBrake.set(signal.leftBrake);
		HardwareAdapter.getInstance().getClimber().rightBrake.set(signal.rightBrake);
		HardwareAdapter.getInstance().getClimber().leftArmLock.set(signal.latchLock ? Value.kForward : Value.kReverse);
		HardwareAdapter.getInstance().getClimber().rightArmLock.set(signal.latchLockRight ? Value.kForward : Value.kReverse);
	}
	
	/**
	 * Helper method for processing a TalonSRXOutput for an SRX
	 */
	private void updateTalonSRX(WPI_TalonSRX talon, TalonSRXOutput output) {
		if(output.getControlMode().equals(ControlMode.Position) || output.getControlMode().equals(ControlMode.Velocity) || output.getControlMode().equals(ControlMode.MotionMagic)) {
			talon.config_kP(output.profile, output.gains.P, 0);
			talon.config_kI(output.profile, output.gains.I, 0);
			talon.config_kD(output.profile, output.gains.D, 0);
			talon.config_kF(output.profile, output.gains.F, 0);
			talon.config_IntegralZone(output.profile, output.gains.izone, 0);
			talon.configClosedloopRamp(output.gains.rampRate, 0);
		}
		if (output.getControlMode().equals(ControlMode.MotionMagic)) {
		    talon.configMotionAcceleration(output.accel, 0);
		    talon.configMotionCruiseVelocity(output.cruiseVel, 0);
		}
		if (output.getControlMode().equals(ControlMode.Velocity)) {
		    talon.configAllowableClosedloopError(output.profile, 0, 0);
		}
		talon.set(output.getControlMode(), output.getSetpoint());
	}
}