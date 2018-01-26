package com.palyrobotics.frc2018.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.palyrobotics.frc2018.config.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Represents all hardware components of the robot.
 * Singleton class. Should only be used in robot package, and 254lib.
 * Subdivides hardware into subsystems.
 * Example call: HardwareAdapter.getInstance().getDrivetrain().getLeftMotor()
 *
 * @author Nihar
 */
public class HardwareAdapter {
	// Hardware components at the top for maintenance purposes, variables and getters at bottom
	/* 
	 * DRIVETRAIN - 6 WPI_TalonSRX's
	 */
	public static class DrivetrainHardware {
		private static DrivetrainHardware instance = new DrivetrainHardware();
		private static DrivetrainHardware getInstance() {
			return instance;
		}

		public final WPI_TalonSRX leftSlave1Talon;
		public final WPI_TalonSRX leftMasterTalon;
		public final WPI_TalonSRX leftSlave2Talon;
		public final WPI_TalonSRX rightSlave1Talon;
		public final WPI_TalonSRX rightMasterTalon;
		public final WPI_TalonSRX rightSlave2Talon;

		public final PigeonIMU gyro;

		public static void resetSensors() {
			instance.gyro.setYaw(0, 0);
			instance.gyro.setFusedHeading(0, 0);
			instance.gyro.setAccumZAngle(0, 0);
			instance.leftMasterTalon.setSelectedSensorPosition(0, 0, 0);
			instance.rightMasterTalon.setSelectedSensorPosition(0, 0, 0);
		}

		private DrivetrainHardware() {
				leftMasterTalon = new WPI_TalonSRX(Constants.kForsetiLeftDriveMasterDeviceID);
				leftSlave1Talon = new WPI_TalonSRX(Constants.kForsetiLeftDriveSlaveDeviceID);
				leftSlave2Talon = new WPI_TalonSRX(Constants.kForsetiLeftDriveOtherSlaveDeviceID);
				rightMasterTalon = new WPI_TalonSRX(Constants.kForsetiRightDriveMasterDeviceID);
				rightSlave1Talon = new WPI_TalonSRX(Constants.kForsetiRightDriveSlaveDeviceID);
				rightSlave2Talon = new WPI_TalonSRX(Constants.kForsetiRightDriveOtherSlaveDeviceID);
				gyro = new PigeonIMU(leftSlave2Talon);
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

		private ClimberHardware() {
			leftVictor = new WPI_VictorSPX(Constants.kForsetiClimberMotorLeftDeviceID);
			rightVictor = new WPI_VictorSPX(Constants.kForsetiClimberMotorRightDeviceID);

			leftArmLock = new DoubleSolenoid(Constants.kForsetiClimberLeftArmBack, Constants.kForsetiClimberLeftArmForward);
			rightArmLock = new DoubleSolenoid(Constants.kForsetiClimberRightArmBack, Constants.kForsetiClimberRightArmForward);

			leftBrake = new Solenoid(Constants.kForsetiClimberLeftBrakeSolenoid);
			rightBrake = new Solenoid(Constants.kForsetiClimberRightBrakeSolenoid);
		}
	}

	// Joysticks for operator interface
	protected static class Joysticks {
		private static Joysticks instance = new Joysticks();

		public static Joysticks getInstance() {
			return instance;
		}

		public final Joystick driveStick = new Joystick(0);
		public final Joystick turnStick = new Joystick(1);
		public final Joystick operatorStick = new Joystick(2);

		private Joysticks() {
		}
	}

	// Wrappers to access hardware groups
	public DrivetrainHardware getDrivetrain() {
		return DrivetrainHardware.getInstance();
	}

	public ClimberHardware getClimber() {
		return ClimberHardware.getInstance();
	}
	
	public Joysticks getJoysticks() {
		return Joysticks.getInstance();
	}
	
	public final PowerDistributionPanel kPDP = new PowerDistributionPanel();

	// Singleton set up
	private static final HardwareAdapter instance = new HardwareAdapter();

	public static HardwareAdapter getInstance() {
		return instance;
	}
}