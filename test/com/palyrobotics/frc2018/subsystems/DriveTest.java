package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.robot.MockRobot;
import com.palyrobotics.frc2018.robot.Robot;
import com.palyrobotics.frc2018.util.CheesyDriveHelper;
import com.palyrobotics.frc2018.util.DriveSignal;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThat;

/**
 * Created by Nihar on 1/22/17. Tests {@link LegacyDrive}
 */
public class DriveTest {
	static Commands commands;
	static RobotState state;
	static Drive drive;

	@BeforeClass
	public static void setUpClass() {
		commands = MockRobot.getCommands();
		state = Robot.getRobotState();
		drive = Drive.getInstance();
	}

	@Before
	public void setUp() {
		drive.resetController();
		Commands.reset();
	}

	@AfterClass
	public static void tearDown() {
		commands = null;
		state = null;
		drive = null;
	}

	@Test
	public void testOffboard() {
		drive.update(commands, state);
		commands.wantedDriveState = Drive.DriveState.OFF_BOARD_CONTROLLER;
		drive.update(commands, state);

		DriveSignal signal = DriveSignal.getNeutralSignal();
		signal.leftMotor.setPercentOutput(0.5);
		signal.rightMotor.setPercentOutput(0.5);
		drive.setTalonSRXController(signal);
		drive.update(commands, state);
		assertThat("not updating correctly", drive.getDriveSignal(), equalTo(signal));
		signal.leftMotor.setPercentOutput(1);
		drive.update(commands, state);
		assertFalse("Signal was updated through external reference!", drive.getDriveSignal() == signal);
	}

	@Test
	public void testPassByReference() {
		drive.update(commands, state);
		commands.wantedDriveState = Drive.DriveState.OFF_BOARD_CONTROLLER;
		drive.update(commands, state);

		DriveSignal newSignal = DriveSignal.getNeutralSignal();
		newSignal.leftMotor.setPercentOutput(1);
		newSignal.rightMotor.setPercentOutput(1);
		drive.setTalonSRXController(newSignal);
		drive.update(commands, state);
		assertThat("not updating correctly", drive.getDriveSignal(), equalTo(newSignal));
	}

	@Test
	public void testNeutral() throws Exception {
		drive.setNeutral();
		assertThat("Drive output not neutral!", drive.getDriveSignal(), equalTo(DriveSignal.getNeutralSignal()));
		//TODO: Undo neutral and try again
	}

	@Test
	public void testChezyDrive() {
		commands.wantedDriveState = Drive.DriveState.CHEZY;
		drive.update(commands, state);
		assertThat("Did not sucessfully set CheesyDrive", drive.getController(), equalTo(CheesyDriveHelper.class));
	}

	@Test
	public void testOpenLoop() {
		commands.wantedDriveState = Drive.DriveState.OPEN_LOOP;
		drive.update(commands, state);
		assertThat("Drive output not corresponding to driveSetpoint", commands.robotSetpoints.drivePowerSetpoint.orElse(null), equalTo(drive.getDriveSignal()));
	}
}
