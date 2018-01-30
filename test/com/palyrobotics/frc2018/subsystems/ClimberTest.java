package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.robot.MockRobot;
import com.palyrobotics.frc2018.robot.Robot;
import org.junit.Test;

import static com.palyrobotics.frc2018.subsystems.Climber.LockState;
import static com.palyrobotics.frc2018.subsystems.Climber.MotionSubstate;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertThat;

import org.junit.Before;

public class ClimberTest {
	Climber climber;
	RobotState robotState;
	Commands commands;
	
	@Before
	public void setUp() {
		climber = Climber.getInstance();
		robotState = MockRobot.getRobotState();
		commands = MockRobot.getCommands();

	}
	
	@Test
	public void testClimber() {
		commands.wantedClimbMovement = MotionSubstate.MOVING;
		climber.update(commands, robotState);
		assertThat("Climber State Set Correctly", climber.getMotionSubstate(), equalTo(MotionSubstate.MOVING));

		commands.wantedLockState = LockState.LOCKED;
		climber.update(commands, robotState);
		assertThat("Climber State Set Correctly", climber.getLock(), equalTo(LockState.LOCKED));

	}
}
