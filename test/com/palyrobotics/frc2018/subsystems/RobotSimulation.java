package com.palyrobotics.frc2018.subsystems;

import java.util.logging.Level;

import com.palyrobotics.frc2018.util.logger.DataLogger;

public class RobotSimulation {	

	private static RobotSimulation instance = new RobotSimulation();
	
	public static RobotSimulation getInstance() {
		return instance;
	}
	
	private DriveSimulation mDriveSimulation = DriveSimulation.getInstance();
	private ElevatorSimulation mElevatorSimulation = ElevatorSimulation.getInstance();
	private IntakeSimulation mIntakeSimulation = IntakeSimulation.getInstance();
	
	private double nominal_voltage = 12.8;
	private double voltage = nominal_voltage;
	private double r_int = 0.013;
	private double last_current = 0.0;
	
	public void step() {
		voltage = nominal_voltage - last_current * r_int;
		mElevatorSimulation.step();
		mDriveSimulation.step();
		mIntakeSimulation.step();
		last_current = mDriveSimulation.getLeftCurrent()*mDriveSimulation.getLeftMotorOutputPercent()
				+ mDriveSimulation.getRightCurrent()*mDriveSimulation.getRightMotorOutputPercent()
				+ mElevatorSimulation.getCurrent()*mElevatorSimulation.getMotorOutputPercent();
	}
	
	public void logState() {
		DataLogger.getInstance().logData(Level.FINE, "voltage", voltage);
		DataLogger.getInstance().logData(Level.FINE, "current", last_current);
		mDriveSimulation.logState();
		mElevatorSimulation.logState();
		mIntakeSimulation.logState();
	}
	
	public double getBatteryVoltage() {
		return voltage;
	}
	
}
