package com.palyrobotics.frc2018.subsystems;

import java.util.logging.Level;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.MockTalon;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import com.palyrobotics.frc2018.util.logger.DataLogger;

public class ElevatorSimulation extends SubsystemSimulation {
	
	private static ElevatorSimulation instance = new ElevatorSimulation();
	
	public static ElevatorSimulation getInstance() {
		return instance;
	}
	
	//Stall Torque in N m
	static final double kStallTorque = 0.71;
	//Stall Current in Amps
	static final double kStallCurrent = 134;
	//Free Speed in RPM
	static final double kFreeSpeed = 18730;
	//Free Current in Amps
	static final double kFreeCurrent = 0.7;
	//Mass of the Elevator
	static final double kMass = 20.0;

	//Number of motors
	static final double kNumMotors = 2.0;
	//Resistance of the motor
	static final double kResistance = 12.0 / kStallCurrent;
	//Motor velocity constant
	static final double Kv = (kFreeSpeed * 2.0 * Math.PI / 60.0 / (12.0 - kResistance * kFreeCurrent));
	//Torque constant
	static final double Kt = (kNumMotors * kStallTorque) / kStallCurrent;
	//Gear ratio
	static final double kG = 42;
	//Radius of pulley
	static final double kr = 3 * 1.432 / 2.0 / 39.37; //m
	//Acceleration of gravity
	static final double g = 9.8;

	//Control loop time step

	//Max elevator height in inches
	static final double kElevatorHeight = 85;
	
	private MockTalon talon;
	
	private double position = 0;
	private double velocity = 0;
	private double acceleration = 0;
	private double voltage = 0;
	
	public ElevatorSimulation() {
		talon = new MockTalon(position*Constants.kElevatorTicksPerInch, "elevator");
		talon.configClosedloopRamp(0.4);
		talon.configPeakOutput(.9,  -.9);
		talon.configVoltageCompSaturation(14.0);
	}

	//V = I * R + omega / Kv
	//torque = Kt * I

	private double calcAcceleration(double voltage) {
		return (kG * Kt / (kResistance * kMass * kr) * voltage - Kt * kG * kG / (Kv * kResistance * kMass * kr * kr) * velocity / 39.37 - g) * 39.37;
	}

	public boolean getBottomHallEffect() {
		return position >= -0.25 && position <= 0.25;
	}

	public boolean getTopHallEffect() {
		return position <= kElevatorHeight + 0.25 && position >= kElevatorHeight - 0.25;
	}
	
	public void step() {
		talon.update();
		//capped voltage
		voltage = talon.getOutputVoltage();
		double prevVelocity = velocity;
		acceleration = calcAcceleration(voltage);
		velocity += kDt * acceleration;
		position += kDt * velocity + 0.5 * kDt * kDt * acceleration;
		//Hard limits
		if (position >= kElevatorHeight) {
			acceleration = -prevVelocity/kDt;
			velocity = 0;
			position = kElevatorHeight;
		}
		else if (position <= 0) {
			acceleration = -prevVelocity/kDt;
			velocity = 0;
			position = 0;
		}
		talon.pushReading(position*Constants.kElevatorTicksPerInch);
	}
	
	public double getSensorPosition() {
		return talon.getReading();
	}
	
	public double getSensorVelocity() {
		return talon.getRate();
	}

	public double getPosition() {
		return position;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public void set(TalonSRXOutput talonSRXOutput) {
		updateTalonSRX(talon, talonSRXOutput);
	}
	
	public void logState() {
		DataLogger.getInstance().logData(Level.FINE, "elevator_position", position);
	}
	
	public String getState() {
		return position + "," + velocity + "," + acceleration + "," + voltage + "," + talon.getSetpoint() / Constants.kElevatorTicksPerInch;
	}

	public void disable() {
		talon.set(ControlMode.Disabled, 0);
		
	}

}