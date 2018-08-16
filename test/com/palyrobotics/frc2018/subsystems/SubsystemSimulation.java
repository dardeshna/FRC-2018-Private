package com.palyrobotics.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.palyrobotics.frc2018.behavior.RoutineManager;
import com.palyrobotics.frc2018.util.MockTalon;
import com.palyrobotics.frc2018.util.TalonSRXOutput;

public abstract class SubsystemSimulation {
		
	public static final double kDt = 0.001;

	protected void updateTalonSRX(MockTalon talon, TalonSRXOutput output) {
		if(output.getControlMode().equals(ControlMode.Position) || output.getControlMode().equals(ControlMode.Velocity)
				|| output.getControlMode().equals(ControlMode.MotionMagic)) {
			talon.setGains(output.gains);
		}
		if(output.getControlMode().equals(ControlMode.MotionMagic)) {
//			talon.configMotionAcceleration(output.accel, 0);
//			talon.configMotionCruiseVelocity(output.cruiseVel, 0);
		}
		talon.set(output.getControlMode(), output.getSetpoint());
	}
}
