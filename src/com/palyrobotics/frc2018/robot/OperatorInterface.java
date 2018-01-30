package com.palyrobotics.frc2018.robot;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Intake;
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

	protected OperatorInterface() {}

	private JoystickInput mDriveStick = Robot.getRobotState().leftStickInput;
	private JoystickInput mTurnStick = Robot.getRobotState().rightStickInput;
	private JoystickInput mClimberStick = Robot.getRobotState().climberStickInput;
	private JoystickInput mOperatorStick = Robot.getRobotState().operatorStickInput;

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

		/**
		 * Drivetrain controls
		 */
		if(prevCommands.wantedDriveState != Drive.DriveState.OFF_BOARD_CONTROLLER
				&& prevCommands.wantedDriveState != Drive.DriveState.ON_BOARD_CONTROLLER) {
			newCommands.wantedDriveState = Drive.DriveState.CHEZY;
		}

		//More safety
		if(Math.abs(ChezyMath.handleDeadband(mDriveStick.getY(), Constants.kDeadband)) > 0.0
				|| Math.abs(ChezyMath.handleDeadband(mTurnStick.getX(), Constants.kDeadband)) > 0.0) {
			newCommands.wantedDriveState = Drive.DriveState.CHEZY;
		}

		/**
		 * Elevator controls
		 */
		if(Math.abs(ChezyMath.handleDeadband(mOperatorStick.getY(), Constants.kDeadband)) > 0.0) {
			newCommands.wantedElevatorState = Elevator.ElevatorState.MANUAL_POSITIONING;
		} else {
			newCommands.wantedElevatorState = Elevator.ElevatorState.HOLD;
		}

		/**
		 * Climber controls
		 */
		if(Math.abs(ChezyMath.handleDeadband(mClimberStick.getY(), Constants.kDeadband)) > 0.0) {
			newCommands.wantedClimbMovement = Climber.MotionSubstate.MOVING;
		} else {
			newCommands.wantedClimbMovement = Climber.MotionSubstate.LOCKED;
		}

		if(mClimberStick.getTriggerPressed()) {
			newCommands.wantedLockState = Climber.LockState.LOCKED;
		}
		if(mClimberStick.getButtonPressed(2)) {
			newCommands.wantedLockState = Climber.LockState.UNLOCKED;
		}
		if(mClimberStick.getButtonPressed(4)) {
			newCommands.wantedClimbSide = Climber.Side.LEFT;
		}
		if(mClimberStick.getButtonPressed(5)) {
			newCommands.wantedClimbSide = Climber.Side.RIGHT;
		}

		/**
		 * Intake controls
		 */

		//Operator intake control
		if(prevCommands.wantedIntakeOpenCloseState == Intake.OpenCloseState.CLOSED && mOperatorStick.getButtonPressed(2)) {
			newCommands.wantedIntakeUpDownState = Intake.UpDownState.UP;
		} else if(mOperatorStick.getButtonPressed(3)) {
			newCommands.wantedIntakeUpDownState = Intake.UpDownState.DOWN;
		}
		
		if(prevCommands.wantedIntakeUpDownState == Intake.UpDownState.DOWN && mOperatorStick.getTriggerPressed()) {
			newCommands.wantedIntakingState = Intake.IntakeState.EXPELLING;
		} else {
			newCommands.wantedIntakingState = Intake.IntakeState.IDLE;
		}

		if(mOperatorStick.getButtonPressed(4)) {
			newCommands.wantedIntakeOpenCloseState = Intake.OpenCloseState.CLOSED;
		} else if(prevCommands.wantedIntakeUpDownState == Intake.UpDownState.DOWN && mOperatorStick.getButtonPressed(5)) {
			newCommands.wantedIntakeOpenCloseState = Intake.OpenCloseState.OPEN;
		}

		//Driver intake control
		if(mTurnStick.getButtonPressed(3)) {
			newCommands.wantedIntakingState = Intake.IntakeState.INTAKING;
		} else if(mTurnStick.getButtonPressed(4)) {
			newCommands.wantedIntakingState = Intake.IntakeState.EXPELLING;
		}

		return newCommands;
	}
}