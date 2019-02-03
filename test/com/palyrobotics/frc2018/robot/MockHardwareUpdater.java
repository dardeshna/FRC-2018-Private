package com.palyrobotics.frc2018.robot;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Optional;

import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.DriveSimulation;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.ElevatorSimulation;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.subsystems.IntakeSimulation;
import com.palyrobotics.frc2018.subsystems.RobotSimulation;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import com.palyrobotics.frc2018.util.trajectory.Kinematics;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;
import com.palyrobotics.frc2018.util.trajectory.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Should only be used in robot package.
 */
class MockHardwareUpdater extends HardwareUpdater {
	
	private Drive mDrive;
	private Climber mClimber;
	private Elevator mElevator;
	private Intake mIntake;

	protected MockHardwareUpdater(Drive drive, Climber climber, Elevator elevator, Intake intake) {
		super(drive, climber, elevator, intake);
		this.mDrive = drive;
		this.mClimber = climber;
		this.mElevator = elevator;
		this.mIntake = intake;
	}
	
	private RobotSimulation mRobotSimulation = RobotSimulation.getInstance();
	private DriveSimulation mDriveSimulation = DriveSimulation.getInstance();
	private ElevatorSimulation mElevatorSimulation = ElevatorSimulation.getInstance();
	private IntakeSimulation mIntakeSimulation = IntakeSimulation.getInstance();
	
	private PrintWriter dumper;
	
	{
		try {
			dumper = new PrintWriter("tmp");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private double lastVelocity = 0;
	private double maxA = 0;
	private double maxV = 0;
	
	@Override
	void disableTalons() {
		mDriveSimulation.disable();
		mElevatorSimulation.disable();
	}

	@Override
	void configureHardware() {
	}

	@Override
	/**
	 * Updates all the sensor data taken from the hardware
	 */
	void updateState(RobotState robotState) {
		
		
		robotState.leftControlMode = mDriveSimulation.getLeftControlMode();
		robotState.rightControlMode = mDriveSimulation.getRightControlMode();

		
//		robotState.leftStickInput.update(HardwareAdapter.getInstance().getJoysticks().driveStick);
//		robotState.rightStickInput.update(HardwareAdapter.getInstance().getJoysticks().turnStick);
//		if(Constants.operatorXBoxController) {
//			robotState.operatorXboxControllerInput.update(HardwareAdapter.getInstance().getJoysticks().operatorXboxController);
//		} else {
//			robotState.climberStickInput.update(HardwareAdapter.getInstance().getJoysticks().climberStick);
//			robotState.operatorJoystickInput.update(HardwareAdapter.getInstance().getJoysticks().operatorJoystick);
//		}
		
		switch(robotState.leftControlMode) {
			//Fall through
			case Position:
			case Velocity:
			case MotionProfileArc:
			case MotionProfile:
//			case MotionMagicArc:
			case MotionMagic:
//				robotState.leftSetpoint = leftMasterTalon.getClosedLoopTarget(0);
				break;
			case Current:
//				robotState.leftSetpoint = leftMasterTalon.getOutputCurrent();
				break;
			//Fall through
			case Follower:
			case PercentOutput:
				robotState.leftSetpoint = mDriveSimulation.getLeftMotorOutputPercent();
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
//			case MotionMagicArc:
			case MotionMagic:
//				robotState.rightSetpoint = rightMasterTalon.getClosedLoopTarget(0);
				break;
			case Current:
//				robotState.rightSetpoint = rightMasterTalon.getOutputCurrent();
				break;
			//Fall through
			case Follower:
			case PercentOutput:
				robotState.rightSetpoint = mDriveSimulation.getRightMotorOutputPercent();
				break;
			default:
				break;
		}

		robotState.drivePose.lastHeading = robotState.drivePose.heading;
		robotState.drivePose.heading = mDriveSimulation.getSensorAngle();
		robotState.drivePose.headingVelocity = (robotState.drivePose.heading - robotState.drivePose.lastHeading) / Constants.kNormalLoopsDt;

		robotState.drivePose.lastLeftEnc = robotState.drivePose.leftEnc;
		robotState.drivePose.leftEnc = mDriveSimulation.getLeftSensorPosition();
		robotState.drivePose.leftEncVelocity = mDriveSimulation.getLeftSensorVelocity();
		robotState.drivePose.lastRightEnc = robotState.drivePose.rightEnc;
		robotState.drivePose.rightEnc = mDriveSimulation.getRightSensorPosition();
		robotState.drivePose.rightEncVelocity = mDriveSimulation.getRightSensorVelocity();

//		if(leftMasterTalon.getControlMode().equals(ControlMode.MotionMagic)) {
//			robotState.drivePose.leftMotionMagicPos = Optional.of(leftMasterTalon.getActiveTrajectoryPosition());
//			robotState.drivePose.leftMotionMagicVel = Optional.of(leftMasterTalon.getActiveTrajectoryVelocity());
//		} else {
//			robotState.drivePose.leftMotionMagicPos = Optional.empty();
//			robotState.drivePose.leftMotionMagicVel = Optional.empty();
//		}
//
//		if(rightMasterTalon.getControlMode().equals(ControlMode.MotionMagic)) {
//			robotState.drivePose.rightMotionMagicPos = Optional.of(rightMasterTalon.getActiveTrajectoryPosition());
//			robotState.drivePose.rightMotionMagicVel = Optional.of(rightMasterTalon.getActiveTrajectoryVelocity());
//		} else {
//			robotState.drivePose.rightMotionMagicPos = Optional.empty();
//			robotState.drivePose.rightMotionMagicVel = Optional.empty();
//		}

//		System.out.println("bot: " + robotState.elevatorBottomHFX);
//		System.out.println("left: " + robotState.drivePose.leftEnc);
//		System.out.println("right: " + robotState.drivePose.rightEnc);
//		System.out.println("gyro : " + robotState.drivePose.heading);

//		Ultrasonic mUltrasonic = HardwareAdapter.getInstance().getIntake().ultrasonic;
//		robotState.mReadings.add(mUltrasonic.getRangeInches());
//		if(robotState.mReadings.size() > 10) {
//			robotState.mReadings.remove();
//		}
//
//		robotState.mSortedReadings = new ArrayList<>(robotState.mReadings);
//		Collections.sort(robotState.mSortedReadings);
//		robotState.cubeDistance = robotState.mSortedReadings.get(robotState.mSortedReadings.size() / 2);
//
//		if(robotState.cubeDistance < Constants.kIntakeCubeInchTolerance) {
//		    if(HardwareAdapter.getInstance().getMiscellaneousHardware().pdp.getCurrent(Constants.kForsetiIntakeMasterDeviceID) > Constants.kIntakeCurrentThreshold) {
//                robotState.hasCube = true;
//            }
//		} else {
//			robotState.hasCube = false;
//		}
		
		robotState.hasCube = mIntakeSimulation.hasCube();

//		System.out.println("elevator: " + robotState.elevatorPosition);
//        System.out.println("left: " + robotState.drivePose.leftEnc);
//        System.out.println("right: " + robotState.drivePose.rightEnc);

		robotState.drivePose.leftError = Optional.of(mDriveSimulation.getLeftClosedLoopError());
		robotState.drivePose.rightError = Optional.of(mDriveSimulation.getRightClosedLoopError());

		double time = Timer.getFPGATimestamp();
//		double time = System.currentTimeMillis();

		//Rotation2d gyro_angle = Rotation2d.fromRadians((right_distance - left_distance) * Constants.kTrackScrubFactor
		///Constants.kTrackEffectiveDiameter);
		Rotation2d gyro_angle = Rotation2d.fromDegrees(robotState.drivePose.heading);
		Rotation2d gyro_velocity = Rotation2d.fromDegrees(robotState.drivePose.headingVelocity);

		RigidTransform2d odometry = robotState.generateOdometryFromSensors((robotState.drivePose.leftEnc - robotState.drivePose.lastLeftEnc) / Constants.kDriveTicksPerInch,
				(robotState.drivePose.rightEnc - robotState.drivePose.lastRightEnc) / Constants.kDriveTicksPerInch, gyro_angle);

		RigidTransform2d.Delta velocity = Kinematics.forwardKinematics(robotState.drivePose.leftEncVelocity / Constants.kDriveSpeedUnitConversion,
				robotState.drivePose.rightEncVelocity / Constants.kDriveSpeedUnitConversion, gyro_velocity.getRadians());

		robotState.addObservations(time, odometry, velocity);
		
//		System.out.println(odometry.getTranslation());
		//System.out.println("Odometry = " + odometry.getTranslation().getX());
//		System.out.println("Velocity = " + velocity.dx);
//		System.out.println("Gyro angle = " + robotState.drivePose.heading);
//		System.out.println("Latest field to vehicle = " + robotState.getLatestFieldToVehicle().toString());
//		System.out.println("Encoder estimate = " + left_distance);

		double cv = (robotState.drivePose.leftEncVelocity + robotState.drivePose.rightEncVelocity)/2 * 1/Constants.kDriveSpeedUnitConversion;

		this.maxV = Math.max(this.maxV, cv);
		this.maxA = Math.max(this.maxA, (cv - lastVelocity)/.02);
//		System.out.println("Max V " + maxV);
//		System.out.println("Max A " + maxA);

//        //Update compressor pressure
//        robotState.compressorPressure = HardwareAdapter.getInstance().getMiscellaneousHardware().compressorSensor.getVoltage() * Constants.kForsetiCompressorVoltageToPSI; //TODO: Implement the constant!
//
//        //Update battery voltage
//        PowerDistributionPanel pdp = HardwareAdapter.getInstance().getMiscellaneousHardware().pdp;
//        robotState.totalCurrentDraw = pdp.getTotalCurrent() - pdp.getCurrent(Constants.kForsetiCompressorDeviceID); //TODO: Implement this!

		//Update elevator sensors
		
		
		robotState.elevatorPosition = mElevatorSimulation.getSensorPosition();
//		System.out.println(robotState.elevatorPosition);
		robotState.elevatorVelocity = mElevatorSimulation.getSensorVelocity();
		robotState.elevatorBottomHFX = mElevatorSimulation.getBottomHallEffect();
		robotState.elevatorTopHFX = mElevatorSimulation.getTopHallEffect();
//		System.out.println("Robot State Position:" + robotState.elevatorPosition);
//		System.out.println("Robot State Velocity:" + robotState.elevatorVelocity);
//		System.out.println("Robot State Bottom HFX:" + robotState.elevatorBottomHFX);
//		System.out.println("Robot State Top HFX:" + robotState.elevatorTopHFX);

		
		robotState.hasElevatorStickyFaults = false;
	}

	@Override
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
		mDriveSimulation.set(mDrive.getDriveSignal());
	}
	
	/**
	 * Updates the elevator
	 */
	private void updateElevator() {

		if(mElevator.getIsAtTop() && mElevator.movingUpwards()) {
			TalonSRXOutput elevatorHoldOutput = new TalonSRXOutput();
			elevatorHoldOutput.setPercentOutput(Constants.kElevatorHoldVoltage);
			mElevatorSimulation.set(elevatorHoldOutput);
		} else {
			mElevatorSimulation.set(mElevator.getOutput());
		}

	}

	private void updateClimber() {
		
	}

	/**
	 * Updates the intake
	 */
	private void updateIntake() {
		mIntakeSimulation.set(mIntake.getWheelState(), mIntake.getOpenCloseState(), mIntake.getUpDownState());
	}
	
	private void updateMiscellaneousHardware() {

    }

	@Override
	void enableBrakeMode() {

	}

	@Override
	void disableBrakeMode() {

	}

	void logSimulations() {
		mRobotSimulation.logState();
	}
	
	void updateSimulations() {
		mRobotSimulation.step();
		dumper.println(mDriveSimulation.getState() + "," + mElevatorSimulation.getState());
	}
	
}