package com.palyrobotics.frc2018.robot;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.MockCommands;
import com.palyrobotics.frc2018.config.MockRobotState;
import com.palyrobotics.frc2018.config.RobotState;

/**
 * Created by EricLiu on 11/12/17.
 */
public class MockRobot extends Robot {
	private static MockRobotState robotState = new MockRobotState();
	private static MockCommands commands = MockCommands.getInstance();
	public static MockRobotState getRobotState() {
		return robotState;
	};
	
	public static MockCommands getCommands() {
		return commands;
	};
}
