package com.palyrobotics.frc2018.config;

import java.util.Optional;

import com.palyrobotics.frc2018.config.Commands.Setpoints;
import com.palyrobotics.frc2018.util.DriveSignal;

public class MockCommands extends Commands {
	
	private static MockCommands instance = new MockCommands();

	public static MockCommands getInstance() {
		return instance;
	}
	
	private MockCommands() {
		super();
	}
	
	public static class MockSetpoints extends Setpoints {
		public void setDrivePowerSetpoint(DriveSignal driveSignal) {
			this.drivePowerSetpoint = Optional.of(driveSignal);
		}
	}
	public MockSetpoints robotSetpoints = new MockSetpoints();
	
	public boolean cancelCurrentRoutines = false;
}