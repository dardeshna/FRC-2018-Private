package com.palyrobotics.frc2018.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.palyrobotics.frc2018.config.Constants;
import edu.wpi.first.wpilibj.*;

/**
 * Represents all hardware components of the robot. Singleton class. Should only be used in robot package, and 254lib. Subdivides hardware into subsystems.
 * Example call: HardwareAdapter.getInstance().getDrivetrain().getLeftMotor()
 *
 * @author Nihar
 */
public class HardwareAdapter {
	//Hardware components at the top for maintenance purposes, variables and getters at bottom
	/*
	 * DRIVETRAIN - 2 WPI_TalonSRX's and 4 WPI_VictorSPX's
	 */
	public static class DrivetrainHardware {
		private static DrivetrainHardware instance = new DrivetrainHardware();

		private static DrivetrainHardware getInstance() {
			return instance;
		}

		public final WPI_TalonSRX leftMasterTalon;
		public final WPI_VictorSPX leftSlave1Victor;
		public final WPI_VictorSPX leftSlave2Victor;
		public final WPI_TalonSRX rightMasterTalon;
		public final WPI_VictorSPX rightSlave1Victor;
		public final WPI_VictorSPX rightSlave2Victor;

		public final PigeonIMU gyro;

		public static void resetSensors() {
			instance.gyro.setYaw(0, 0);
			instance.gyro.setFusedHeading(0, 0);
			instance.gyro.setAccumZAngle(0, 0);
			instance.leftMasterTalon.setSelectedSensorPosition(0, 0, 0);
			instance.rightMasterTalon.setSelectedSensorPosition(0, 0, 0);
		}

		protected DrivetrainHardware() {
			leftMasterTalon = new WPI_TalonSRX(Constants.kForsetiLeftDriveMasterDeviceID);
			leftSlave1Victor = new WPI_VictorSPX(Constants.kForsetiLeftDriveSlaveDeviceID);
			leftSlave2Victor = new WPI_VictorSPX(Constants.kForsetiLeftDriveOtherSlaveDeviceID);
			rightMasterTalon = new WPI_TalonSRX(Constants.kForsetiRightDriveMasterDeviceID);
			rightSlave1Victor = new WPI_VictorSPX(Constants.kForsetiRightDriveSlaveDeviceID);
			rightSlave2Victor = new WPI_VictorSPX(Constants.kForsetiRightDriveOtherSlaveDeviceID);
			gyro = new PigeonIMU(leftMasterTalon);
		}
	}

	/*
	 * Climber - 2 WPI_TalonSRX's, 2 DoubleSolenoids, 2 Solenoids
	 */
	public static class ClimberHardware {
		private static ClimberHardware instance = new ClimberHardware();

		private static ClimberHardware getInstance() {
			return instance;
		}

		public final WPI_VictorSPX leftVictor;
		public final WPI_VictorSPX rightVictor;

		public final DoubleSolenoid leftArmLock;
		public final DoubleSolenoid rightArmLock;

		public final Solenoid leftBrake;
		public final Solenoid rightBrake;

		protected ClimberHardware() {
			leftVictor = new WPI_VictorSPX(Constants.kForsetiClimberMotorLeftDeviceID);
			rightVictor = new WPI_VictorSPX(Constants.kForsetiClimberMotorRightDeviceID);

			leftArmLock = new DoubleSolenoid(Constants.kForsetiClimberLeftArmBack, Constants.kForsetiClimberLeftArmForward);
			rightArmLock = new DoubleSolenoid(Constants.kForsetiClimberRightArmBack, Constants.kForsetiClimberRightArmForward);

			leftBrake = new Solenoid(Constants.kForsetiClimberLeftBrakeSolenoid);
			rightBrake = new Solenoid(Constants.kForsetiClimberRightBrakeSolenoid);
		}
	}

	/**
	 * Elevator - 2 WPI_TalonSRX's, 2 HFX (DigitalInputs)
	 */
	public static class ElevatorHardware {
		private static ElevatorHardware instance = new ElevatorHardware();

		private static ElevatorHardware getInstance() {
			return instance;
		}

		public final WPI_TalonSRX elevatorMasterTalon;
		public final WPI_TalonSRX elevatorSlaveTalon;

		public final DigitalInput bottomHFX;
		public final DigitalInput topHFX;

		protected ElevatorHardware() {
			elevatorMasterTalon = new WPI_TalonSRX(Constants.kForsetiElevatorMasterTalonID);
			elevatorSlaveTalon = new WPI_TalonSRX(Constants.kForsetiElevatorSlaveTalonID);

			topHFX = new DigitalInput(Constants.kForsetiTopElevatorHFXID);
			bottomHFX = new DigitalInput(Constants.kForsetiBottomElevatorHFXID);
		}
	}

	/**
	 * Intake - 2 WPI_TalonSRX's, 2 DoubleSolenoids, 1 Distance Sensor (AnalogInput)
	 */
	public static class IntakeHardware {
		private static IntakeHardware instance = new IntakeHardware();

		private static IntakeHardware getInstance() {
			return instance;
		}

		public final WPI_TalonSRX masterTalon;
		public final WPI_TalonSRX slaveTalon;
		public final DoubleSolenoid upDownSolenoid;
		public final Solenoid openCloseSolenoid;
		public final Solenoid openCloseOtherSolenoid;
		public final AnalogInput distanceSensor;

		protected IntakeHardware() {
			masterTalon = new WPI_TalonSRX(Constants.kForsetiIntakeMasterDeviceID);
			slaveTalon = new WPI_TalonSRX(Constants.kForsetiIntakeSlaveDeviceID);
			upDownSolenoid = new DoubleSolenoid(Constants.kForsetiIntakeUpDownSolenoidForwardID, Constants.kForsetiIntakeUpDownSolenoidReverseID);
			openCloseSolenoid = new Solenoid(Constants.kForsetiIntakeOpenCloseSolenoidForwardID, Constants.kForsetiIntakeOpenCloseSolenoidReverseID);
			openCloseOtherSolenoid = new Solenoid(Constants.kForsetiIntakeOpenCloseOtherSolenoidForwardID, Constants.kForsetiIntakeOpenCloseOtherSolenoidReverseID);
			distanceSensor = new AnalogInput(Constants.kForsetiIntakeDistanceSensorID);
		}
	}

	//Joysticks for operator interface
	public static class Joysticks {
		private static Joysticks instance = new Joysticks();

		private static Joysticks getInstance() {
			return instance;
		}

		public final Joystick driveStick = new Joystick(0);
		public final Joystick turnStick = new Joystick(1);
		public final Joystick climberStick = new Joystick(2);
		public final Joystick operatorStick = new Joystick(3);

		protected Joysticks() {
		}
	}

	//Wrappers to access hardware groups
	public DrivetrainHardware getDrivetrain() {
		return DrivetrainHardware.getInstance();
	}

	public ElevatorHardware getElevator() {
		return ElevatorHardware.getInstance();
	}

	public IntakeHardware getIntake() {
		return IntakeHardware.getInstance();
	}

	public ClimberHardware getClimber() {
		return ClimberHardware.getInstance();
	}

	public Joysticks getJoysticks() {
		return Joysticks.getInstance();
	}

	public final PowerDistributionPanel kPDP = new PowerDistributionPanel();

	//Singleton set up
	private static final HardwareAdapter instance = new HardwareAdapter();

	public static HardwareAdapter getInstance() {
		return instance;
	}
}