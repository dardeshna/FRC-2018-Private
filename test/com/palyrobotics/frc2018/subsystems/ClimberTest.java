package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.robot.MockRobot;
import com.palyrobotics.frc2018.subsystems.Climber.Side;

import org.junit.Before;
import org.junit.Test;

import static com.palyrobotics.frc2018.subsystems.Climber.LockState;
import static com.palyrobotics.frc2018.subsystems.Climber.MotionSubstate;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertThat;

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
	
	@Test
	public void testClimberLeftBrake() {
		commands.wantedClimbMovement = MotionSubstate.MOVING;
		commands.wantedClimbSide = Side.LEFT;
		climber.update(commands, robotState);
		assertThat("Climber Left Brake is On", climber.getSignal().leftBrake, equalTo(false));
		
		commands.wantedClimbMovement = MotionSubstate.LOCKED;
		commands.wantedClimbSide = Side.LEFT;
		climber.update(commands, robotState);
		assertThat("Climber Left Brake is not On", climber.getSignal().leftBrake, equalTo(true));
	}
	
	@Test
	public void testClimberRightBrake() {
		commands.wantedClimbMovement = MotionSubstate.MOVING;
		commands.wantedClimbSide = Side.RIGHT;
		climber.update(commands, robotState);
		assertThat("Climber Right Brake is On", climber.getSignal().rightBrake, equalTo(false));
		
		commands.wantedClimbMovement = MotionSubstate.LOCKED;
		commands.wantedClimbSide = Side.RIGHT;
		climber.update(commands, robotState);
		assertThat("Climber Right Brake is not On", climber.getSignal().rightBrake, equalTo(true));
	}
}
