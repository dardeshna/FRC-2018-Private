package com.palyrobotics.frc2018.util;

import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.subsystems.SubsystemSimulation;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class MockTalon {
	
	private static int measurementWindow = 100;
	private static int rollingAvgPeriod = 64;
	
	private String name;
	private Gains gains;
	private ControlMode mode = ControlMode.Disabled;
	private int[] sensorReadings = new int[measurementWindow+rollingAvgPeriod];
	private int prevSensorRate = 0;
	private int sensorRate = 0;
	private int setpoint;
	private int error;
	private int iAccum;
	private boolean reset;
	private int output;
	private int prevOutput;
	private double forwardsPeakOutput = 1;
	private double reversePeakOutput = 1;
	private double rampRate = 0;
	private double deadband = 0.04;
	private double voltageCompSaturation = 12.0;
	
	
	public MockTalon(double reading) {
		this(reading, "null");
	}
	public MockTalon(double reading, String name) {
		this.name = name;
		sensorReadings = new int[measurementWindow+rollingAvgPeriod];
		for (int i = 0; i < sensorReadings.length; i++) {
			sensorReadings[i] = (int) reading;
		}
	}
	
	public void setGains(Gains gains) {
		this.gains = gains;
	}
	
	public void set(ControlMode mode, double setpoint) {
//		System.out.println("Control Mode: " + mode + "\nSetpoint: " + setpoint);
		if (this.mode != mode) {
			this.reset = true;
		}
		this.mode = mode;
		this.setpoint = (int) setpoint;
	}
	
	public void update() {
		this.prevOutput = output;
		if (this.mode == ControlMode.PercentOutput) {
			output = (int) (this.setpoint * 1023);
			error = 0;
		}
		else if (this.mode == ControlMode.Position) {
			pid(sensorReadings[0], sensorReadings[1]);
		}
		else if (this.mode == ControlMode.Velocity) {
			pid(sensorRate, prevSensorRate);
		}
		else {
			output = 0;
			error = 0;
		}
		if (rampRate != 0) {
			if (Math.signum(output)*(output - prevOutput) > 1023/rampRate * SubsystemSimulation.kDt) {
				output = (int)(prevOutput + Math.signum(output - prevOutput) * 1023/rampRate * SubsystemSimulation.kDt);
			}
		}
	}
	
	private void pid(int pos, int prevPos) {
		error = setpoint - pos;
		if (reset) {
			iAccum = 0;
		}
 		if (gains.izone == 0 ||  Math.abs(error) < gains.izone) {
			iAccum += error;
		}
 		else {
 			iAccum = 0;
 		}
		
 		int dErr = prevPos - pos;
 		if (reset) dErr = 0;
 		 		
 		if (reset) reset = false;
 		
 		this.output = (int) Math.max(Math.min(1023*forwardsPeakOutput, error*gains.P + dErr*gains.D + iAccum*gains.I + setpoint*gains.F), -1023*reversePeakOutput);
 		
	}

	public void pushReading(double reading) {
		
			int[] temp = new int[measurementWindow+rollingAvgPeriod];
			for (int i = 0; i < sensorReadings.length-1; i++) {
				temp[i+1] = sensorReadings[i];
			}
			temp[0] = (int) reading;
			sensorReadings = temp;
		
		
		int avg = 0;
		for (int i = 0; i < rollingAvgPeriod; i++) {
			avg += sensorReadings[i] - sensorReadings[i+measurementWindow];
		}
		avg /= rollingAvgPeriod;
		prevSensorRate = sensorRate;
		sensorRate = avg;
		
	}
	
	public void resetSensor() {
		int currentReading = sensorReadings[0];
		for (int i = 0; i < sensorReadings.length-1; i++) {
			sensorReadings[i] = sensorReadings[i] - currentReading;
		}
	}
	
	public int getReading() {
		return sensorReadings[0];
	}
	
	public int getRate() {
		return sensorRate;
	}
	
	public int getSetpoint() {
		return setpoint;
	}
	
	public void configPeakOutput(double forwards, double reverse) {
		forwardsPeakOutput = forwards;
		reversePeakOutput = reverse;
	}
	
	public void configClosedloopRamp(double rampRate) {
		this.rampRate = rampRate;
	}
	
	public void configNeutralDeadband(double deadband) {
		this.deadband = deadband;
	}
	
	public void configVoltageCompSaturation(double voltageCompSaturation) {
		this.voltageCompSaturation = voltageCompSaturation;
		
	}
	
	public double getMotorOutputVoltage() {
		return getMotorOutputVoltage(12.0);
	}
	
	public double getMotorOutputVoltage(double batteryVoltage) {

		if (Math.abs(output) <= deadband*1023) {
			return 0;
		}
		else {
			return Math.max(-batteryVoltage, Math.min(batteryVoltage, output/1023.0 * voltageCompSaturation));
		}
	}
	
	public double getMotorOutputPercent() {
		return getMotorOutputPercent(12.0);
	}
	
	public double getMotorOutputPercent(double batteryVoltage) {
		return getMotorOutputVoltage(batteryVoltage)/batteryVoltage;
	}
	
	public int getClosedLoopError() {
		return error;
	}

	public ControlMode getControlMode() {
		return this.mode;
	}

	
	

}
