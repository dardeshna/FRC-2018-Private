package com.palyrobotics.frc2018.robot;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.*;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.ChezyMath;
import com.palyrobotics.frc2018.util.JoystickInput;

import java.util.ArrayList;

/**
 * Used to produce Commands {@link Commands} from human input Singleton class. Should only be used in robot package.
 * 
 * @author Nihar
 *
 */
public class OperatorInterface {
	private static OperatorInterface instance = new OperatorInterface();

	public static OperatorInterface getInstance() {
		return instance;
	}

	private boolean operatorButtonFourPressable = true;

	protected OperatorInterface() {
	}

	private JoystickInput mDriveStick = Robot.getRobotState().leftStickInput;
	private JoystickInput mTurnStick = Robot.getRobotState().rightStickInput;
	private JoystickInput mClimberStick = Robot.getRobotState().climberStickInput;
	private JoystickInput mOperatorStick = Robot.getRobotState().operatorStickInput;

	/**
	 * Helper method to only add routines that aren't already in wantedRoutines
	 * 
	 * @param commands
	 *            Current set of commands being modified
	 * @param wantedRoutine
	 *            Routine to add to the commands
	 * @return whether or not wantedRoutine was successfully added
	 */
	private boolean addWantedRoutine(Commands commands, Routine wantedRoutine) {
		for(Routine routine : commands.wantedRoutines) {
			if(routine.getClass().equals(wantedRoutine.getClass())) {
				return false;
			}
		}
		commands.wantedRoutines.add(wantedRoutine);
		return true;
	}

	/**
	 * Returns modified commands
	 * 
	 * @param prevCommands
	 */
	public Commands updateCommands(Commands prevCommands) {
		Commands newCommands = prevCommands.copy();

		/**
		 * Drivetrain controls
		 */
		if(prevCommands.wantedDriveState != Drive.DriveState.OFF_BOARD_CONTROLLER && prevCommands.wantedDriveState != Drive.DriveState.ON_BOARD_CONTROLLER) {
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
//			newCommands.wantedElevatorState = Elevator.ElevatorState.HOLD;
		}
		if(mOperatorStick.getButtonPressed(11)) {
			newCommands.disableElevatorScaling = true;
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
		} else if(mClimberStick.getButtonPressed(2)) {
			newCommands.wantedLockState = Climber.LockState.UNLOCKED;
		}

		if(mClimberStick.getButtonPressed(4)) {
			newCommands.wantedClimbSide = Climber.Side.LEFT;
			newCommands.addWantedRoutine(new SequentialRoutine(new ArrayList<Routine>() {{
				new IntakeCloseRoutine();
				new IntakeUpRoutine();
			}}));
		} else if(mClimberStick.getButtonPressed(5)) {
			newCommands.wantedClimbSide = Climber.Side.RIGHT;
			newCommands.addWantedRoutine(new SequentialRoutine(new ArrayList<Routine>() {{
				new IntakeCloseRoutine();
				new IntakeUpRoutine();
			}}));
		}

		/**
		 * Intake controls
		 */

		//Operator intake control
		//Up/Down block
		 if(mOperatorStick.getButtonPressed(4)) {
		 	if(operatorButtonFourPressable) {
				if(prevCommands.wantedIntakeUpDownState == Intake.UpDownState.DOWN) {
					newCommands.addWantedRoutine(new IntakeUpRoutine());
				} else {
					newCommands.addWantedRoutine(new IntakeDownRoutine());
				}
			}

			operatorButtonFourPressable = false;
		}

		//Close/Open block, cannot be executed along with up/down
		else if(mOperatorStick.getButtonPressed(3)) {
			 newCommands.addWantedRoutine(new IntakeNeutralRoutine());
		 } else if(mOperatorStick.getButtonPressed(5)) {
			newCommands.addWantedRoutine(new IntakeCloseRoutine());
		} else if(mOperatorStick.getButtonPressed(6)) {
			newCommands.addWantedRoutine(new IntakeOpenRoutine());
		} else {
			operatorButtonFourPressable = true;
		}

		//Intake wheel logic block
		if(mOperatorStick.getTriggerPressed()) {
			newCommands.wantedIntakingState = Intake.WheelState.EXPELLING;
		} else if(mOperatorStick.getButtonPressed(2)) {
			newCommands.wantedIntakingState = Intake.WheelState.INTAKING;
		} else {
			newCommands.wantedIntakingState = Intake.WheelState.IDLE;
		}

		//Driver intake control
		//Override above intake wheel logic block
		//Default idle state is in above logic block
		if(mTurnStick.getButtonPressed(3)) {
			newCommands.wantedIntakingState = Intake.WheelState.INTAKING;
		} else if(mTurnStick.getButtonPressed(4)) {
			newCommands.wantedIntakingState = Intake.WheelState.EXPELLING;
		}

		if(mTurnStick.getButtonPressed(5)) {
			newCommands.addWantedRoutine(new IntakeCloseRoutine());
		} else if(mTurnStick.getButtonPressed(6)) {
			newCommands.addWantedRoutine(new IntakeNeutralRoutine());
		}

		return newCommands;
	}
}