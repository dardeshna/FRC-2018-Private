package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.MockTalon;
import com.palyrobotics.frc2018.util.physics.DCMotorTransmission;
import com.palyrobotics.frc2018.util.physics.DifferentialDrive;

public class DriveSimulation extends SubsystemSimulation {
	
	private static DriveSimulation instance = new DriveSimulation();
	
	public static DriveSimulation getInstance() {
		return instance;
	}
	

	static double speed_per_volt_ = 0;
	static double torque_per_volt_ = 0;
	static double friction_voltage_ = 0;
	
	static double kRobotLinearInteria;
	static double kRobotAngularInertia;
	static double kRobotAngularDrag;
	static double kRobotWheelRadius = Constants.kDriveWheelDiameterInches / 2.0 / 39.3701;
	static double kRobotEffectiveWheelBaseRadius = Constants.kTrackEffectiveDiameter / 2.0 / 39.3701;
	
	private DifferentialDrive mModel;
	private MockTalon rightTalon;
	private MockTalon leftTalon;
	
	public DriveSimulation() {
		 final DCMotorTransmission transmission = new DCMotorTransmission(speed_per_volt_, torque_per_volt_, friction_voltage_);
	     mModel = new DifferentialDrive(kRobotLinearInteria, kRobotAngularInertia, kRobotAngularDrag, kRobotWheelRadius, kRobotEffectiveWheelBaseRadius, transmission, transmission);
	     rightTalon = new MockTalon(0);
	     leftTalon = new MockTalon(0);
	}
	
	
	//Stall Torque in N m
	

	double current_time = 0;

	public void step(double voltage) {			
			final double current_dt = kDt;
//			position_ += current_dt * velocity_;
//			velocity_ += current_dt * GetAcceleration(voltage);
	}

	public void set(DriveSignal signal) {
		
		
	}

	public void resetSensors() {
		// TODO Auto-generated method stub
		
	}
}