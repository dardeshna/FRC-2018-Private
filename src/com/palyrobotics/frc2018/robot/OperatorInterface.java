package com.palyrobotics.frc2018.robot;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Elevator.ElevatorState;
import com.palyrobotics.frc2018.util.ChezyMath;
import com.palyrobotics.frc2018.util.JoystickInput;

/**
 * Used to produce Commands {@link Commands} from human input
 * Singleton class. Should only be used in robot package.
 * @author Nihar
 *
 */
public class OperatorInterface {
	private static OperatorInterface instance = new OperatorInterface();

	public static OperatorInterface getInstance() {
		return instance;
	}

	private OperatorInterface() {}

	private JoystickInput mDriveStick = Robot.getRobotState().leftStickInput;
	private JoystickInput mTurnStick = Robot.getRobotState().rightStickInput;
	private JoystickInput mOperatorStick = Robot.getRobotState().operatorStickInput;
	private JoystickInput mElevatorStick = Robot.getRobotState().elevatorStickInput;

	/**
	 * Helper method to only add routines that aren't already in wantedRoutines
	 * @param commands Current set of commands being modified
	 * @param wantedRoutine Routine to add to the commands
	 * @return whether or not wantedRoutine was successfully added
	 */
	private boolean addWantedRoutine(Commands commands, Routine wantedRoutine) {
		for (Routine routine : commands.wantedRoutines) {
			if (routine.getClass().equals(wantedRoutine.getClass())) {
				return false;
			}
		}
		commands.wantedRoutines.add(wantedRoutine);
		return true;
	}

	/**
	 * Returns modified commands
	 * @param prevCommands
	 */
	public Commands updateCommands(Commands prevCommands) {
		Commands newCommands = prevCommands.copy();

		if(prevCommands.wantedDriveState != Drive.DriveState.OFF_BOARD_CONTROLLER
				&& prevCommands.wantedDriveState != Drive.DriveState.ON_BOARD_CONTROLLER) {
			newCommands.wantedDriveState = Drive.DriveState.CHEZY;
		}
		
		// Handle elevator states
		if (Math.abs(ChezyMath.handleDeadband(mElevatorStick.getY(), Constants.kDeadband)) > 0.0) {
			newCommands.wantedElevatorState = Elevator.ElevatorState.MANUAL_POSITIONING;
		} else {
			newCommands.wantedElevatorState = Elevator.ElevatorState.HOLD;
		}

		if (Math.abs(ChezyMath.handleDeadband(mOperatorStick.getY(), Constants.kDeadband)) > 0.0) {
			newCommands.wantedClimbMovement = Climber.MotionSubstate.MOVING;
		} else {
			newCommands.wantedClimbMovement = Climber.MotionSubstate.LOCKED;
		}

		if (mOperatorStick.getTriggerPressed()) {
			newCommands.wantedLockState = (Climber.LockState.LOCKED == prevCommands.wantedLockState) ?
											Climber.LockState.UNLOCKED : Climber.LockState.LOCKED;
		}

		if(prevCommands.wantedUpDownState == Intake.UpDownState.DOWN && mOperatorStick.getTriggerPressed()) {
			newCommands.wantedIntakeState = Intake.IntakeState.INTAKING;
		} else if(prevCommands.wantedUpDownState == Intake.UpDownState.DOWN && mOperatorStick.getButtonPressed(3)) {
			newCommands.wantedIntakeState = Intake.IntakeState.EXPELLING;
		} else {
			newCommands.wantedIntakeState = Intake.IntakeState.IDLE;
		}

		if(prevCommands.wantedOpenCloseState == Intake.OpenCloseState.CLOSED && mOperatorStick.getButtonPressed(8)) {
			newCommands.wantedUpDownState = Intake.UpDownState.UP;
		} else if(prevCommands.wantedOpenCloseState == Intake.OpenCloseState.CLOSED && mOperatorStick.getButtonPressed(9)) {
			newCommands.wantedUpDownState = Intake.UpDownState.DOWN;
		}

		if(prevCommands.wantedUpDownState == Intake.UpDownState.DOWN && prevCommands.wantedOpenCloseState == Intake.OpenCloseState.CLOSED && mOperatorStick.getButtonPressed(4)) {
			newCommands.wantedOpenCloseState = Intake.OpenCloseState.OPEN;
		} else if(prevCommands.wantedUpDownState == Intake.UpDownState.DOWN && prevCommands.wantedOpenCloseState == Intake.OpenCloseState.OPEN && mOperatorStick.getButtonPressed(4)) {
			newCommands.wantedOpenCloseState = Intake.OpenCloseState.CLOSED;
		}

		return newCommands;
	}
}