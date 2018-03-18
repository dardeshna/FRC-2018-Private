package com.palyrobotics.frc2018.robot;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.*;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.ChezyMath;
import com.palyrobotics.frc2018.util.JoystickInput;
import com.palyrobotics.frc2018.util.XboxInput;

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
	private boolean operatorButtonTwoPressable = true;

	private JoystickInput mDriveStick = Robot.getRobotState().leftStickInput;
	private JoystickInput mTurnStick = Robot.getRobotState().rightStickInput;
	private JoystickInput mClimberStick = Robot.getRobotState().climberStickInput;
	private JoystickInput mOperatorJoystick = null;
	private XboxInput mOperatorXboxController = null;

	protected OperatorInterface() {
		if(Constants.operatorXBoxController) {
			mOperatorXboxController = Robot.getRobotState().operatorXboxControllerInput;
		} else {
			mOperatorJoystick = Robot.getRobotState().operatorJoystickInput;
		}
	}

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

		newCommands.cancelCurrentRoutines = false;

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

		if(Constants.operatorXBoxController) {
			/**
			 * Elevator controls
			 */
			if(Math.abs(ChezyMath.handleDeadband(mOperatorXboxController.getRightY(), 0.05)) > 0.0) {
				newCommands.wantedElevatorState = Elevator.ElevatorState.MANUAL_POSITIONING;
			} else {
				newCommands.wantedElevatorState = Elevator.ElevatorState.HOLD;
			}
			// Start button
			if(mOperatorXboxController.getButtonPressed(8)) {
				newCommands.disableElevatorScaling = true;
			}

			/**
			 * Intake controls
			 */

			//Operator intake control
			if(mOperatorXboxController.getdPadUp()) {
				newCommands.addWantedRoutine(new IntakeUpRoutine());
				newCommands.cancelCurrentRoutines = true;
			} else if(mOperatorXboxController.getdPadRight()) {
				newCommands.addWantedRoutine(new IntakeNeutralRoutine());
				newCommands.cancelCurrentRoutines = true;
			} else if(mOperatorXboxController.getdPadDown()) {
				newCommands.addWantedRoutine(new IntakeDownRoutine());
				newCommands.cancelCurrentRoutines = true;
			} else if(mOperatorXboxController.getdPadLeft()) {
				newCommands.addWantedRoutine(new IntakeCloseRoutine());
				newCommands.cancelCurrentRoutines = true;
			} else if(mOperatorXboxController.getRightBumper()) {
				newCommands.addWantedRoutine(new IntakeOpenRoutine());
				newCommands.cancelCurrentRoutines = true;
			}

			//Intake wheel logic block
			if(mOperatorXboxController.getRightTrigger() > 0.0) {
				newCommands.wantedIntakingState = Intake.WheelState.EXPELLING;
				newCommands.customIntakeSpeed = true;
				newCommands.cancelCurrentRoutines = true;
			} else if(mOperatorXboxController.getLeftBumper()) {
				if (operatorButtonTwoPressable) {
					ArrayList<Routine> intakeThenUp = new ArrayList<>();
					intakeThenUp.add(new IntakeSensorStopRoutine(Intake.WheelState.INTAKING, 115.0));
					intakeThenUp.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorCubeInTransitPositionInches, 0.5));
					newCommands.cancelCurrentRoutines = true;
					newCommands.addWantedRoutine(new SequentialRoutine(intakeThenUp));
				}
				operatorButtonTwoPressable = false;
			} else if(mOperatorXboxController.getButtonA()) {
				newCommands.wantedIntakingState = Intake.WheelState.VAULT_EXPELLING;
				newCommands.cancelCurrentRoutines = true;
			} else if(mOperatorXboxController.getLeftTrigger() > 0.0) {
				newCommands.wantedIntakingState = Intake.WheelState.INTAKING;
				newCommands.customIntakeSpeed = true;
				newCommands.cancelCurrentRoutines = true;
			} else {
				newCommands.customIntakeSpeed = false;
				operatorButtonTwoPressable = true;
				newCommands.wantedIntakingState = Intake.WheelState.IDLE;
			}

		} else {
			/**
			 * Elevator controls
			 */
			if (Math.abs(ChezyMath.handleDeadband(mOperatorJoystick.getY(), 0.02)) > 0.0) {
				newCommands.wantedElevatorState = Elevator.ElevatorState.MANUAL_POSITIONING;
			} else {
				newCommands.wantedElevatorState = Elevator.ElevatorState.HOLD;
			}
			if (mOperatorJoystick.getButtonPressed(11)) {
				newCommands.disableElevatorScaling = true;
			}

			/**
			 * Intake controls
			 */

			//Operator intake control
			//Up/Down block
			if (mOperatorJoystick.getButtonPressed(4)) {
				if (operatorButtonFourPressable) {
					if (prevCommands.wantedIntakeUpDownState == Intake.UpDownState.DOWN) {
						newCommands.addWantedRoutine(new IntakeUpRoutine());
						newCommands.cancelCurrentRoutines = true;
					} else {
						newCommands.addWantedRoutine(new IntakeDownRoutine());
						newCommands.cancelCurrentRoutines = true;
					}
				}

				operatorButtonFourPressable = false;
			}

			//Close/Open block, cannot be executed along with up/down
			else if (mOperatorJoystick.getButtonPressed(3)) {
				newCommands.addWantedRoutine(new IntakeNeutralRoutine());
				newCommands.cancelCurrentRoutines = true;
			} else if (mOperatorJoystick.getButtonPressed(5)) {
				newCommands.addWantedRoutine(new IntakeCloseRoutine());
				newCommands.cancelCurrentRoutines = true;
			} else if (mOperatorJoystick.getButtonPressed(6)) {
				newCommands.addWantedRoutine(new IntakeOpenRoutine());
				newCommands.cancelCurrentRoutines = true;
			} else {
				operatorButtonFourPressable = true;
			}

			//Intake wheel logic block
			if (mOperatorJoystick.getTriggerPressed()) {
				newCommands.wantedIntakingState = Intake.WheelState.EXPELLING;
				newCommands.cancelCurrentRoutines = true;
			} else if (mOperatorJoystick.getButtonPressed(2)) {
				if (operatorButtonTwoPressable) {
					ArrayList<Routine> intakeThenUp = new ArrayList<>();
					intakeThenUp.add(new IntakeSensorStopRoutine(Intake.WheelState.INTAKING, 115.0));
					intakeThenUp.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorCubeInTransitPositionInches, 0.5));
					newCommands.cancelCurrentRoutines = true;
					newCommands.addWantedRoutine(new SequentialRoutine(intakeThenUp));
				}
				operatorButtonTwoPressable = false;
			} else if (mOperatorJoystick.getButtonPressed(10)) {
				newCommands.wantedIntakingState = Intake.WheelState.VAULT_EXPELLING;
				newCommands.cancelCurrentRoutines = true;
			} else if (mOperatorJoystick.getButtonPressed(9)) {
				newCommands.wantedIntakingState = Intake.WheelState.INTAKING;
				newCommands.cancelCurrentRoutines = true;
			} else {
				operatorButtonTwoPressable = true;
				newCommands.wantedIntakingState = Intake.WheelState.IDLE;
			}
		}

		/**
		 * Climber controls
		 */
		if(Math.abs(ChezyMath.handleDeadband(mClimberStick.getY(), 0.1)) > 0.0) {
			newCommands.wantedClimbMovement = Climber.MotionSubstate.MOVING;

			ArrayList<Routine> stowIntakeRoutine = new ArrayList<Routine>();
			stowIntakeRoutine.add(new IntakeCloseRoutine());
			stowIntakeRoutine.add(new IntakeUpRoutine());
			newCommands.addWantedRoutine(new SequentialRoutine(stowIntakeRoutine));
		} else {
			newCommands.wantedClimbMovement = Climber.MotionSubstate.LOCKED;
		}

		if (mClimberStick.getTriggerPressed()) {
			newCommands.wantedLockState = Climber.LockState.LOCKED;

			ArrayList<Routine> stowIntakeRoutine = new ArrayList<Routine>();
			stowIntakeRoutine.add(new IntakeCloseRoutine());
			stowIntakeRoutine.add(new IntakeUpRoutine());
			newCommands.addWantedRoutine(new SequentialRoutine(stowIntakeRoutine));
		} else if (mClimberStick.getButtonPressed(2)) {
			newCommands.wantedLockState = Climber.LockState.UNLOCKED;

			ArrayList<Routine> stowIntakeRoutine = new ArrayList<Routine>();
			stowIntakeRoutine.add(new IntakeCloseRoutine());
			stowIntakeRoutine.add(new IntakeUpRoutine());
			newCommands.addWantedRoutine(new SequentialRoutine(stowIntakeRoutine));
		}

		//Driver intake control
		//Override above intake wheel logic block
		//Default idle state is in above logic block
		if(mTurnStick.getButtonPressed(3)) {
			newCommands.wantedIntakingState = Intake.WheelState.INTAKING;
			newCommands.cancelCurrentRoutines = true;
		} else if(mTurnStick.getButtonPressed(4)) {
			newCommands.wantedIntakingState = Intake.WheelState.VAULT_EXPELLING;
			newCommands.cancelCurrentRoutines = true;
		}

		return newCommands;
	}
}