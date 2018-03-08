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
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.ClimberSignal;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.trajectory.Kinematics;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;
import com.palyrobotics.frc2018.util.trajectory.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.util.Optional;
import java.util.logging.Level;

/**
 * Should only be used in robot package.
 */
class HardwareUpdater {

	//Subsystem references
	private Drive mDrive;
	private Climber mClimber;
	private Elevator mElevator;
	private Intake mIntake;

	private int intakeStallCounter = 10;
	private int intakeFreeSpinCounter = 0;

	/**
	 * Hardware Updater for Forseti
	 */
	protected HardwareUpdater(Drive drive, Climber climber, Elevator elevator, Intake intake) {
		this.mDrive = drive;
		this.mClimber = climber;
		this.mElevator = elevator;
		this.mIntake = intake;
	}

	/**
	 * Initialize all hardware
	 */
	void initHardware() {
		Logger.getInstance().logRobotThread(Level.INFO, "Init hardware");
		configureHardware();
	}

	void disableTalons() {
		Logger.getInstance().logRobotThread(Level.INFO, "Disabling talons");

		//Disable drivetrain talons
		HardwareAdapter.getInstance().getDrivetrain().leftMasterTalon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().leftSlave1Victor.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().leftSlave2Victor.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().leftSlave3Victor.set(ControlMode.Disabled, 0);

		HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().rightSlave1Victor.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().rightSlave2Victor.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getDrivetrain().rightSlave3Victor.set(ControlMode.Disabled, 0);

		//Disable climber talons
		HardwareAdapter.getInstance().getClimber().climberVictor.set(ControlMode.Disabled, 0);

		//Disable elevator talons
		HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getElevator().elevatorSlaveTalon.set(ControlMode.Disabled, 0);

		//Disable intake talons
		HardwareAdapter.getInstance().getIntake().masterTalon.set(ControlMode.Disabled, 0);
		HardwareAdapter.getInstance().getIntake().slaveTalon.set(ControlMode.Disabled, 0);
	}

	void configureHardware() {
		configureDriveHardware();
		configureClimberHardware();
		configureElevatorHardware();
		configureIntakeHardware();
	}

	void configureDriveHardware() {
		PigeonIMU gyro = HardwareAdapter.getInstance().getDrivetrain().gyro;
		gyro.setYaw(0, 0);
		gyro.setFusedHeading(0, 0);

		WPI_TalonSRX leftMasterTalon = HardwareAdapter.getInstance().getDrivetrain().leftMasterTalon;
		WPI_VictorSPX leftSlave1Victor = HardwareAdapter.getInstance().getDrivetrain().leftSlave1Victor;
        WPI_VictorSPX leftSlave2Victor = HardwareAdapter.getInstance().getDrivetrain().leftSlave2Victor;
        WPI_VictorSPX leftSlave3Victor = HardwareAdapter.getInstance().getDrivetrain().leftSlave3Victor;

        WPI_TalonSRX rightMasterTalon = HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon;
		WPI_VictorSPX rightSlave1Victor = HardwareAdapter.getInstance().getDrivetrain().rightSlave1Victor;
        WPI_VictorSPX rightSlave2Victor = HardwareAdapter.getInstance().getDrivetrain().rightSlave2Victor;
        WPI_VictorSPX rightSlave3Victor = HardwareAdapter.getInstance().getDrivetrain().rightSlave3Victor;

		//Enable all talons' brake mode and disables forward and reverse soft
		leftMasterTalon.setNeutralMode(NeutralMode.Brake);
		leftSlave1Victor.setNeutralMode(NeutralMode.Brake);
        leftSlave2Victor.setNeutralMode(NeutralMode.Brake);
        leftSlave3Victor.setNeutralMode(NeutralMode.Brake);
        rightMasterTalon.setNeutralMode(NeutralMode.Brake);
		rightSlave1Victor.setNeutralMode(NeutralMode.Brake);
        rightSlave2Victor.setNeutralMode(NeutralMode.Brake);
        rightSlave3Victor.setNeutralMode(NeutralMode.Brake);

        leftMasterTalon.enableVoltageCompensation(true);
        leftSlave1Victor.enableVoltageCompensation(true);
        leftSlave2Victor.enableVoltageCompensation(true);
        leftSlave3Victor.enableVoltageCompensation(true);
        rightMasterTalon.enableVoltageCompensation(true);
        rightSlave1Victor.enableVoltageCompensation(true);
        rightSlave2Victor.enableVoltageCompensation(true);
        rightSlave3Victor.enableVoltageCompensation(true);

		leftMasterTalon.configVoltageCompSaturation(14, 0);
		leftSlave1Victor.configVoltageCompSaturation(14, 0);
        leftSlave2Victor.configVoltageCompSaturation(14, 0);
        leftSlave3Victor.configVoltageCompSaturation(14, 0);
        rightMasterTalon.configVoltageCompSaturation(14, 0);
		rightSlave1Victor.configVoltageCompSaturation(14, 0);
        rightSlave2Victor.configVoltageCompSaturation(14, 0);
        rightSlave3Victor.configVoltageCompSaturation(14, 0);

        leftMasterTalon.configForwardSoftLimitEnable(false, 0);
		leftMasterTalon.configReverseSoftLimitEnable(false, 0);
		leftSlave1Victor.configForwardSoftLimitEnable(false, 0);
		leftSlave1Victor.configReverseSoftLimitEnable(false, 0);
        leftSlave2Victor.configForwardSoftLimitEnable(false, 0);
        leftSlave2Victor.configReverseSoftLimitEnable(false, 0);
        leftSlave3Victor.configForwardSoftLimitEnable(false, 0);
        leftSlave3Victor.configReverseSoftLimitEnable(false, 0);

		rightMasterTalon.configForwardSoftLimitEnable(false, 0);
		rightMasterTalon.configReverseSoftLimitEnable(false, 0);
		rightSlave1Victor.configForwardSoftLimitEnable(false, 0);
		rightSlave1Victor.configReverseSoftLimitEnable(false, 0);
        rightSlave2Victor.configForwardSoftLimitEnable(false, 0);
        rightSlave2Victor.configReverseSoftLimitEnable(false, 0);
        rightSlave3Victor.configForwardSoftLimitEnable(false, 0);
        rightSlave3Victor.configReverseSoftLimitEnable(false, 0);

		leftMasterTalon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		leftMasterTalon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
		leftSlave1Victor.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		leftSlave1Victor.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
        leftSlave2Victor.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
        leftSlave2Victor.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
        leftSlave3Victor.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
        leftSlave3Victor.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);

		rightMasterTalon.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		rightMasterTalon.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
        rightSlave1Victor.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
        rightSlave1Victor.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
		rightSlave2Victor.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
		rightSlave2Victor.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);
        rightSlave3Victor.configPeakOutputForward(Constants.kDriveMaxClosedLoopOutput, 0);
        rightSlave3Victor.configPeakOutputReverse(-Constants.kDriveMaxClosedLoopOutput, 0);

		//Configure master talon feedback devices
		leftMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		leftMasterTalon.setSensorPhase(true);
		rightMasterTalon.setSensorPhase(true);

		rightMasterTalon.overrideLimitSwitchesEnable(false);

		leftMasterTalon.setStatusFramePeriod(0, 5, 0);
		rightMasterTalon.setStatusFramePeriod(0, 5, 0);

		leftMasterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 0);
		rightMasterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 0);

		leftMasterTalon.configVelocityMeasurementWindow(16, 0);
		rightMasterTalon.configVelocityMeasurementWindow(16, 0);

		//Zero encoders
		leftMasterTalon.setSelectedSensorPosition(0, 0, 0);
		rightMasterTalon.setSelectedSensorPosition(0, 0, 0);

		leftMasterTalon.configClosedloopRamp(0.2, 0);
		rightMasterTalon.configClosedloopRamp(0.2, 0);

		leftMasterTalon.configOpenloopRamp(0.15, 0);
		rightMasterTalon.configOpenloopRamp(0.15, 0);

		//Reverse right side
		rightMasterTalon.setInverted(true);
		rightSlave1Victor.setInverted(true);
		rightSlave2Victor.setInverted(true);
		rightSlave3Victor.setInverted(true);

		//Set slave victors to follower mode
		leftSlave1Victor.follow(leftMasterTalon);
        leftSlave2Victor.follow(leftMasterTalon);
        leftSlave3Victor.follow(leftMasterTalon);
        rightSlave1Victor.follow(rightMasterTalon);
        rightSlave2Victor.follow(rightMasterTalon);
        rightSlave3Victor.follow(rightMasterTalon);
    }

	void configureElevatorHardware() {
		WPI_TalonSRX masterTalon = HardwareAdapter.getInstance().getElevator().elevatorMasterTalon;
		WPI_TalonSRX slaveTalon = HardwareAdapter.getInstance().getElevator().elevatorSlaveTalon;

		masterTalon.setInverted(true);
		slaveTalon.setInverted(false);

		slaveTalon.follow(masterTalon);

		masterTalon.enableVoltageCompensation(true);
		slaveTalon.enableVoltageCompensation(true);

		masterTalon.configVoltageCompSaturation(14, 0);
		slaveTalon.configVoltageCompSaturation(14, 0);

		masterTalon.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon.getDeviceID(), 0);
//		masterTalon.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon.getDeviceID(), 0);

		masterTalon.overrideLimitSwitchesEnable(true);
		slaveTalon.overrideLimitSwitchesEnable(true);

		masterTalon.configPeakOutputForward(1, 0);
		masterTalon.configPeakOutputReverse(-1, 0);
		slaveTalon.configPeakOutputForward(1, 0);
		slaveTalon.configPeakOutputReverse(-1, 0);

		masterTalon.configClosedloopRamp(0.4, 0);
		masterTalon.configOpenloopRamp(0.4, 0);

		masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		masterTalon.setSensorPhase(true);

		//Zero encoders
		masterTalon.setSelectedSensorPosition(0, 0, 0);
	}

	void configureClimberHardware() {
		WPI_VictorSPX climberRight = HardwareAdapter.getInstance().getClimber().climberVictor;

		climberRight.enableVoltageCompensation(true);

		climberRight.configVoltageCompSaturation(14, 0);

		climberRight.setNeutralMode(NeutralMode.Brake);

		climberRight.configForwardSoftLimitEnable(false, 0);
		climberRight.configReverseSoftLimitEnable(false, 0);
	}

	void configureIntakeHardware() {

		WPI_TalonSRX masterTalon = HardwareAdapter.getInstance().getIntake().masterTalon;
		WPI_TalonSRX slaveTalon = HardwareAdapter.getInstance().getIntake().slaveTalon;

		masterTalon.setNeutralMode(NeutralMode.Brake);
		slaveTalon.setNeutralMode(NeutralMode.Brake);

		masterTalon.enableVoltageCompensation(true);
		slaveTalon.enableVoltageCompensation(true);

		masterTalon.configVoltageCompSaturation(14, 0);
		slaveTalon.configVoltageCompSaturation(14, 0);

		//Disables forwards and reverse soft limits
		masterTalon.configForwardSoftLimitEnable(false, 0);
		masterTalon.configReverseSoftLimitEnable(false, 0);
		slaveTalon.configForwardSoftLimitEnable(false, 0);
		slaveTalon.configReverseSoftLimitEnable(false, 0);

		//Reverse right side
		masterTalon.setInverted(false);
		slaveTalon.setInverted(false);

		//Set slave talons to follower mode
        slaveTalon.follow(masterTalon);
	}

	void configureMiscellaneousHardware() {
		PowerDistributionPanel pdp = HardwareAdapter.getInstance().getMiscellaneousHardware().pdp;
		LiveWindow.disableTelemetry(pdp);
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
		robotState.climberStickInput.update(HardwareAdapter.getInstance().getJoysticks().climberStick);
		robotState.operatorStickInput.update(HardwareAdapter.getInstance().getJoysticks().operatorStick);

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
			default:
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
			default:
				break;
		}

		PigeonIMU gyro = HardwareAdapter.getInstance().getDrivetrain().gyro;
		if(gyro != null) {
			robotState.drivePose.heading = gyro.getFusedHeading();
			robotState.drivePose.headingVelocity = (robotState.drivePose.heading - robotState.drivePose.lastHeading) / Constants.kNormalLoopsDt;
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

		if(leftMasterTalon.getControlMode().equals(ControlMode.MotionMagic)) {
			robotState.drivePose.leftMotionMagicPos = Optional.of(leftMasterTalon.getActiveTrajectoryPosition());
			robotState.drivePose.leftMotionMagicVel = Optional.of(leftMasterTalon.getActiveTrajectoryVelocity());
		} else {
			robotState.drivePose.leftMotionMagicPos = Optional.empty();
			robotState.drivePose.leftMotionMagicVel = Optional.empty();
		}

		if(rightMasterTalon.getControlMode().equals(ControlMode.MotionMagic)) {
			robotState.drivePose.rightMotionMagicPos = Optional.of(rightMasterTalon.getActiveTrajectoryPosition());
			robotState.drivePose.rightMotionMagicVel = Optional.of(rightMasterTalon.getActiveTrajectoryVelocity());
		} else {
			robotState.drivePose.rightMotionMagicPos = Optional.empty();
			robotState.drivePose.rightMotionMagicVel = Optional.empty();
		}

		double masterCurrent = HardwareAdapter.getInstance().getIntake().masterTalon.getOutputCurrent();
		double slaveCurrent = HardwareAdapter.getInstance().getIntake().slaveTalon.getOutputCurrent();

		if(masterCurrent >= Constants.kIntakeMasterStallCurrent
				&& slaveCurrent >= Constants.kIntakeSlaveStallCurrent
				&& mIntake.getWheelState() == Intake.WheelState.INTAKING) {
			intakeStallCounter++;
		} else {
			intakeStallCounter = 0;
		}

		if(masterCurrent > Constants.kIntakeMasterIdleCurrent
				&& masterCurrent <= Constants.kIntakeMasterExpelCurrent
				&& slaveCurrent > Constants.kIntakeSlaveIdleCurrent
				&& slaveCurrent <= Constants.kIntakeSlaveExpelCurrent
				&& mIntake.getWheelState() == Intake.WheelState.EXPELLING) {
			intakeFreeSpinCounter++;
		} else if(masterCurrent > Constants.kIntakeMasterIdleCurrent
				&& masterCurrent <= Constants.kIntakeMasterVaultExpelCurrent
				&& slaveCurrent > Constants.kIntakeSlaveIdleCurrent
				&& slaveCurrent <= Constants.kIntakeSlaveVaultExpelCurrent
				&& mIntake.getWheelState() == Intake.WheelState.VAULT_EXPELLING) {
			intakeFreeSpinCounter++;
		} else if(masterCurrent > Constants.kIntakeMasterIdleCurrent
				&& masterCurrent <= Constants.kIntakeMasterIntakingCurrent
				&& slaveCurrent > Constants.kIntakeSlaveIdleCurrent
				&& slaveCurrent <= Constants.kIntakeSlaveIntakingCurrent
				&& mIntake.getWheelState() == Intake.WheelState.INTAKING) {
			intakeFreeSpinCounter++;
		} else {
			intakeFreeSpinCounter = 0;
		}

		if(intakeStallCounter >= Constants.kIntakeStallCounterThreshold) {
			robotState.hasCube = true;
		} else if(mIntake.getOpenCloseState() == Intake.OpenCloseState.OPEN || intakeFreeSpinCounter >= Constants.kIntakeFreeCounterThreshold){
			robotState.hasCube = false;
		}

		robotState.drivePose.leftError = Optional.of(leftMasterTalon.getClosedLoopError(0));
		robotState.drivePose.rightError = Optional.of(rightMasterTalon.getClosedLoopError(0));

		double time = Timer.getFPGATimestamp();

		//Rotation2d gyro_angle = Rotation2d.fromRadians((right_distance - left_distance) * Constants.kTrackScrubFactor
		///Constants.kTrackEffectiveDiameter);
		Rotation2d gyro_angle = Rotation2d.fromDegrees(robotState.drivePose.heading);
		Rotation2d gyro_velocity = Rotation2d.fromDegrees(robotState.drivePose.headingVelocity);

		RigidTransform2d odometry = robotState.generateOdometryFromSensors((robotState.drivePose.leftEnc - robotState.drivePose.lastLeftEnc) / Constants.kDriveTicksPerInch,
				(robotState.drivePose.rightEnc - robotState.drivePose.lastRightEnc) / Constants.kDriveTicksPerInch, gyro_angle);

		RigidTransform2d.Delta velocity = Kinematics.forwardKinematics(robotState.drivePose.leftEncVelocity / Constants.kDriveSpeedUnitConversion,
				robotState.drivePose.rightEncVelocity / Constants.kDriveSpeedUnitConversion, gyro_velocity.getRadians());

		robotState.addObservations(time, odometry, velocity);

		//System.out.println("Odometry = " + odometry.getTranslation().getX());
		//System.out.println("Velocity = " + velocity.dx);
//		System.out.println("Gyro angle = " + robotState.drivePose.heading);
//		System.out.println("Latest field to vehicle = " + robotState.getLatestFieldToVehicle().toString());
//		System.out.println("Encoder estimate = " + left_distance);

//        //Update compressor pressure
//        robotState.compressorPressure = HardwareAdapter.getInstance().getMiscellaneousHardware().compressorSensor.getVoltage() * Constants.kForsetiCompressorVoltageToPSI; //TODO: Implement the constant!
//
//        //Update battery voltage
//        PowerDistributionPanel pdp = HardwareAdapter.getInstance().getMiscellaneousHardware().pdp;
//        robotState.totalCurrentDraw = pdp.getTotalCurrent() - pdp.getCurrent(Constants.kForsetiCompressorDeviceID); //TODO: Implement this!

		//Update elevator sensors
		robotState.elevatorPosition = HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.getSelectedSensorPosition(0);
		robotState.elevatorVelocity = HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.getSelectedSensorVelocity(0);
		robotState.elevatorBottomHFX = HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon.getSensorCollection().isRevLimitSwitchClosed();
		robotState.elevatorTopHFX = HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon.getSensorCollection().isFwdLimitSwitchClosed();
		StickyFaults elevatorStickyFaults = new StickyFaults();
		HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.clearStickyFaults(0);
		HardwareAdapter.getInstance().getElevator().elevatorMasterTalon.getStickyFaults(elevatorStickyFaults);
		robotState.hasElevatorStickyFaults = elevatorStickyFaults.hasAnyFault();
	}

	/**
	 * Updates the hardware to run with output values of subsystems
	 */
	void updateHardware() {
		updateDrivetrain();
		updateClimber();
		updateElevator();
		updateIntake();
		updateMiscellaneousHardware();
	}

	/**
	 * Updates the drivetrain Uses TalonSRXOutput and can run off-board control loops through SRX
	 */
	private void updateDrivetrain() {
		updateTalonSRX(HardwareAdapter.getInstance().getDrivetrain().leftMasterTalon, mDrive.getDriveSignal().leftMotor);
		updateTalonSRX(HardwareAdapter.getInstance().getDrivetrain().rightMasterTalon, mDrive.getDriveSignal().rightMotor);
	}


    /**
     * Checks if the compressor should compress and updates it accordingly
     */
	private void updateMiscellaneousHardware() {
	    if(shouldCompress()) {
	        HardwareAdapter.getInstance().getMiscellaneousHardware().compressor.start();
        } else {
            HardwareAdapter.getInstance().getMiscellaneousHardware().compressor.stop();
        }
    }

    /**
     * Runs the compressor only when the pressure too low or the current draw is
     * low enough
     */
    private boolean shouldCompress() {
//        double currentDraw = RobotState.getInstance().totalCurrentDraw;
//        double pressure = RobotState.getInstance().compressorPressure;
//        return currentDraw * pressure < Constants.kForsetiPressureCurrentProductThreshold; //TODO: Implement this!
    	return !(RobotState.getInstance().gamePeriod == RobotState.GamePeriod.AUTO);
    }

	/**
	 * Updates the elevator
	 */
	private void updateElevator() {

		//If climber is up, disable elevator input
		if(mClimber.getClimberPositionEstimate() > Constants.kClimberUpPositionEstimateThreshold) {
			updateTalonSRX(HardwareAdapter.getInstance().getElevator().elevatorMasterTalon, new TalonSRXOutput());
		} else {
			if(mElevator.getIsAtTop() && mElevator.movingUpwards()) {
				TalonSRXOutput elevatorHoldOutput = new TalonSRXOutput();
				elevatorHoldOutput.setPercentOutput(Constants.kElevatorHoldVoltage);
				updateTalonSRX(HardwareAdapter.getInstance().getElevator().elevatorMasterTalon, elevatorHoldOutput);
			} else {
				updateTalonSRX(HardwareAdapter.getInstance().getElevator().elevatorMasterTalon, mElevator.getOutput());
			}
		}
	}

	private void updateClimber() {
		ClimberSignal signal = mClimber.getSignal();
		HardwareAdapter.getInstance().getClimber().climberVictor.set(ControlMode.PercentOutput, signal.velocity);
		HardwareAdapter.getInstance().getClimber().climberBrake.set(!signal.brake);
		HardwareAdapter.getInstance().getClimber().climberArmLock.set(signal.latchLock ? Value.kForward : Value.kReverse);
	}

	/**
	 * Updates the intake
	 */
	private void updateIntake() {
		updateTalonSRX(HardwareAdapter.getInstance().getIntake().masterTalon, mIntake.getTalonOutput());
		HardwareAdapter.getInstance().getIntake().openCloseSolenoid.set(mIntake.getOpenCloseOutput()[0]);
		HardwareAdapter.getInstance().getIntake().openCloseOtherSolenoid.set(mIntake.getOpenCloseOutput()[1]);
		HardwareAdapter.getInstance().getIntake().upDownSolenoid.set(mIntake.getUpDownOutput());
	}

	/**
	 * Helper method for processing a TalonSRXOutput for an SRX
	 */
	private void updateTalonSRX(WPI_TalonSRX talon, TalonSRXOutput output) {
		if(output.getControlMode().equals(ControlMode.Position) || output.getControlMode().equals(ControlMode.Velocity)
				|| output.getControlMode().equals(ControlMode.MotionMagic)) {
			talon.config_kP(output.profile, output.gains.P, 0);
			talon.config_kI(output.profile, output.gains.I, 0);
			talon.config_kD(output.profile, output.gains.D, 0);
			talon.config_kF(output.profile, output.gains.F, 0);
			talon.config_IntegralZone(output.profile, output.gains.izone, 0);
			talon.configClosedloopRamp(output.gains.rampRate, 0);
		}
		if(output.getControlMode().equals(ControlMode.MotionMagic)) {
			talon.configMotionAcceleration(output.accel, 0);
			talon.configMotionCruiseVelocity(output.cruiseVel, 0);
		}
		if(output.getControlMode().equals(ControlMode.Velocity)) {
			talon.configAllowableClosedloopError(output.profile, 0, 0);
		}
		talon.set(output.getControlMode(), output.getSetpoint());
	}
}