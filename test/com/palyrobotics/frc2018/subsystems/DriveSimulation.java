package com.palyrobotics.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.MockTalon;
import com.palyrobotics.frc2018.util.geometry.Pose2d;
import com.palyrobotics.frc2018.util.geometry.Twist2d;
import com.palyrobotics.frc2018.util.physics.DCMotorTransmission;
import com.palyrobotics.frc2018.util.physics.DifferentialDrive;
import com.palyrobotics.frc2018.util.physics.DifferentialDrive.DriveDynamics;
import com.palyrobotics.frc2018.util.physics.DifferentialDrive.WheelState;

public class DriveSimulation extends SubsystemSimulation {
	
	private static DriveSimulation instance = new DriveSimulation();
	
	public static DriveSimulation getInstance() {
		return instance;
	}
	
	static final double gear_ratio = 80.0/12.0 * 60.0/14.0;

	static final double motors_per_gearbox = 4.0;
	static final double speed_per_volt_ = 18730 * 2.0 * Math.PI / 60 / gear_ratio / 12.0;
	static final double torque_per_volt_ = motors_per_gearbox * 0.71 * gear_ratio / 12.0;
	static final double friction_voltage_ = 1.0;
	
	public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    
	static final double kRobotWheelRadius = Constants.kDriveWheelDiameterInches / 2.0 / 39.3701;
	static final double kRobotEffectiveWheelBaseRadius = Constants.kTrackEffectiveDiameter / 2.0 / 39.3701;
	
	static final double voltageCompSaturation = 14.0;
	
	private DifferentialDrive mModel;
	private MockTalon rightTalon;
	private MockTalon leftTalon;
	private WheelState voltage;
	private DriveDynamics driveDynamics = new DriveDynamics();
	private WheelState wheel_position = new WheelState();
	private Pose2d pos = Pose2d.identity();
	
	public DriveSimulation() {
		 final DCMotorTransmission transmission = new DCMotorTransmission(speed_per_volt_, torque_per_volt_, friction_voltage_);
	     mModel = new DifferentialDrive(kRobotLinearInertia, kRobotAngularInertia, kRobotAngularDrag, kRobotWheelRadius, kRobotEffectiveWheelBaseRadius, transmission, transmission);
	     rightTalon = new MockTalon(0, "right");
	     leftTalon = new MockTalon(0, "left");
	     
	     rightTalon.configVoltageCompSaturation(voltageCompSaturation);
	     leftTalon.configVoltageCompSaturation(voltageCompSaturation);
	     
//	     rightTalon.configClosedloopRamp(0.4);
//	     rightTalon.configVoltageCompSaturation(14.0);
//	     
//	     leftTalon.configClosedloopRamp(0.4);
//	     leftTalon.configVoltageCompSaturation(14.0);
	}
	//V = I * R + omega / Kv
	//torque = Kt * I
	
	
	public void step() {
		rightTalon.update();
		leftTalon.update();
		
		voltage = new WheelState(leftTalon.getOutputVoltage(), rightTalon.getOutputVoltage());
		driveDynamics = mModel.solveForwardDynamics(driveDynamics.chassis_velocity, voltage);
		
		driveDynamics.chassis_velocity.linear += driveDynamics.chassis_acceleration.linear*kDt;
		driveDynamics.chassis_velocity.angular += driveDynamics.chassis_acceleration.angular*kDt;
		
		driveDynamics.wheel_velocity.left += driveDynamics.wheel_acceleration.left*kDt;
		driveDynamics.wheel_velocity.right += driveDynamics.wheel_acceleration.right*kDt;
		
		wheel_position.left += driveDynamics.wheel_velocity.left*kDt;
		wheel_position.right += driveDynamics.wheel_velocity.right*kDt;
		
		pos = pos.transformBy(Pose2d.exp(new Twist2d(driveDynamics.chassis_velocity.linear*kDt, 0, driveDynamics.chassis_velocity.angular*kDt)));
		
		leftTalon.pushReading(wheel_position.left*Constants.kDriveWheelDiameterInches/2.0*Constants.kDriveTicksPerInch);
		rightTalon.pushReading(wheel_position.right*Constants.kDriveWheelDiameterInches/2.0*Constants.kDriveTicksPerInch);
	}
	
	public double getLeftSensorPosition() {
		return leftTalon.getReading();
	}
	public double getRightSensorPosition() {
		return rightTalon.getReading();
	}
	
	public double getLeftSensorVelocity() {
		return leftTalon.getRate();
	}
	public double getRightSensorVelocity() {
		return rightTalon.getRate();
	}

	public double getSensorAngle() {
		return pos.getRotation().getDegrees();
	}
	
	public ControlMode getLeftControlMode() {
		return leftTalon.getControlMode();
	}
	public ControlMode getRightControlMode() {
		return rightTalon.getControlMode();
	}
	
	public double getLeftOutput() {
		return voltage.left/voltageCompSaturation;
	}
	public double getRightOutput() {
		return voltage.right/voltageCompSaturation;
	}
	
	public int getLeftClosedLoopError() {
		return leftTalon.getClosedLoopError();
	}
	public int getRightClosedLoopError() {
		return rightTalon.getClosedLoopError();
	}
	
	public void set(DriveSignal signal) {
		updateTalonSRX(leftTalon, signal.leftMotor);
		updateTalonSRX(rightTalon, signal.rightMotor);
	}
	
	public String getState() {
		Pose2d scaledPos = new Pose2d(pos.getTranslation().scale(39.3701), pos.getRotation());
		return scaledPos.toCSV();
	}

	public void resetSensors() {
		pos = Pose2d.identity();
		wheel_position = new WheelState(0,0);
		leftTalon.resetSensor();
		rightTalon.resetSensor();
		
	}


	public void disable() {
		rightTalon.set(ControlMode.Disabled, 0);
		leftTalon.set(ControlMode.Disabled, 0);
		
	}

}