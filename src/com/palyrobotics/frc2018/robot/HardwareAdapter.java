package com.palyrobotics.frc2018.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.XboxController;
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
        public final WPI_VictorSPX leftSlave3Victor;
        public final WPI_TalonSRX rightMasterTalon;
		public final WPI_VictorSPX rightSlave1Victor;
		public final WPI_VictorSPX rightSlave2Victor;
        public final WPI_VictorSPX rightSlave3Victor;

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
			leftSlave1Victor = new WPI_VictorSPX(Constants.kForsetiLeftDriveSlave1DeviceID);
			leftSlave2Victor = new WPI_VictorSPX(Constants.kForsetiLeftDriveSlave2DeviceID);
            leftSlave3Victor = new WPI_VictorSPX(Constants.kForsetiLeftDriveSlave3DeviceID);
            rightMasterTalon = new WPI_TalonSRX(Constants.kForsetiRightDriveMasterDeviceID);
			rightSlave1Victor = new WPI_VictorSPX(Constants.kForsetiRightDriveSlave1DeviceID);
			rightSlave2Victor = new WPI_VictorSPX(Constants.kForsetiRightDriveSlave2DeviceID);
            rightSlave3Victor = new WPI_VictorSPX(Constants.kForsetiRightDriveSlave3DeviceID);

			//Gyro is currently attached to elevator talon as an... emergency provision

			//HAL
//			gyro = new PigeonIMU(new WPI_TalonSRX(Constants.kForsetiElevatorSlaveTalonID));

			//Forseti
			gyro = new PigeonIMU(new WPI_TalonSRX(Constants.kForsetiIntakeSlaveDeviceID));
//			gyro = new PigeonIMU(new WPI_TalonSRX(11));
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

		public final WPI_VictorSPX climberVictor;

		public final DoubleSolenoid climberArmLock;

		final Solenoid climberBrake;

		protected ClimberHardware() {
			climberVictor = new WPI_VictorSPX(Constants.kForsetiClimberMotorDeviceID);

			climberArmLock = new DoubleSolenoid(0, Constants.kForsetiClimberLatchSolenoidReverseID, Constants.kForsetiClimberLatchSolenoidForwardID);

			if (Constants.kRobotName == Constants.RobotName.FORSETI) {
				climberBrake = new Solenoid(1, Constants.kForsetiClimberBrakeSolenoidID);
			}
			else {
				climberBrake = new Solenoid(1, Constants.kHALClimberBrakeSolenoidID);

			}
		}
	}

	/**
	 * Elevator - 2 WPI_TalonSRX's
	 */
	public static class ElevatorHardware {
		private static ElevatorHardware instance = new ElevatorHardware();

		private static ElevatorHardware getInstance() {
			return instance;
		}

		public final WPI_TalonSRX elevatorMasterTalon;
		public final WPI_TalonSRX elevatorSlaveTalon;

		protected ElevatorHardware() {
			elevatorMasterTalon = new WPI_TalonSRX(Constants.kForsetiElevatorMasterTalonID);
			elevatorSlaveTalon = new WPI_TalonSRX(Constants.kForsetiElevatorSlaveTalonID);
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

		protected IntakeHardware() {
			masterTalon = new WPI_TalonSRX(Constants.kForsetiIntakeMasterDeviceID);
			slaveTalon = new WPI_TalonSRX(Constants.kForsetiIntakeSlaveDeviceID);
			upDownSolenoid = new DoubleSolenoid(0, Constants.kForsetiIntakeUpDownSolenoidForwardID, Constants.kForsetiIntakeUpDownSolenoidReverseID);
			if (Constants.kRobotName == Constants.RobotName.FORSETI) {
				openCloseSolenoid = new Solenoid(1, Constants.kForsetiIntakeOpenCloseSolenoidID);
			}
			else {
				openCloseSolenoid = new Solenoid(1, Constants.kHALIntakeOpenCloseSolenoid);

			}
			openCloseOtherSolenoid = new Solenoid(1, Constants.kForsetiIntakeOpenCloseOtherSolenoidID);
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
		public Joystick operatorJoystick = null;
		public XboxController operatorXboxController = null;

		protected Joysticks() {
			if(Constants.operatorXBoxController) {
				operatorXboxController = new XboxController(3, false);
			} else {
				operatorJoystick = new Joystick(3);
			}
		}
	}

    /**
     * Miscellaneous Hardware - Compressor sensor(Analog Input), Compressor, PDP
     */
	public static class MiscellaneousHardware {
	    private static MiscellaneousHardware instance = new MiscellaneousHardware();

	    private static MiscellaneousHardware getInstance() {
	        return instance;
        }

        public final Compressor compressor;
        public final AnalogInput compressorSensor;

        protected MiscellaneousHardware() {
            compressor = new Compressor();
            compressorSensor = new AnalogInput(Constants.kForsetiCompressorSensorDeviceID);
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

	public MiscellaneousHardware getMiscellaneousHardware() {
	    return MiscellaneousHardware.getInstance();
    }

	//Singleton set up
	private static final HardwareAdapter instance = new HardwareAdapter();

	public static HardwareAdapter getInstance() {
		return instance;
	}
}