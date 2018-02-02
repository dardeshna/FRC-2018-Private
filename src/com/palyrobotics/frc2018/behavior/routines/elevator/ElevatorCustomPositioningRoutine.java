package com.palyrobotics.frc2018.behavior.routines.elevator;

import java.util.logging.Level;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.subsystems.Drive.DriveController;
import com.palyrobotics.frc2018.subsystems.Drive.DriveState;
import com.palyrobotics.frc2018.subsystems.Elevator.ElevatorState;
import com.palyrobotics.frc2018.util.CheesyDriveHelper;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.logger.Logger;

public class ElevatorCustomPositioningRoutine extends Routine {

	private enum ElevatorCustomPositioningRoutineState {
		START, //Routine was just called
		WAITING, //If calibration is in progress, wait for it to finish
		MOVING, //Elevator is moving into position
		DONE //Elevator has reached its target
	}
	
	private enum ElevatorPosition {
		TOP, //Elevator moves to topmost position
		SWITCH, //Elevator moves to the height of the switch
		BOTTOM //Elevator moves to bottommost position
	}
	
	private ElevatorCustomPositioningRoutineState mState = ElevatorCustomPositioningRoutineState.START;

	//Which elevator position should move to
	private ElevatorPosition mPosition;
	
	//How long the routine has until it times out (seconds)
	private double mTimeout;
	
	private long mStartTime;
	
	public ElevatorCustomPositioningRoutine(ElevatorPosition position, double timeout) {
		mPosition = position;
		mTimeout = timeout;
	}
	
	@Override
	public void start() {
		mState = ElevatorCustomPositioningRoutineState.START;
	}

	/**
	 * Processes the routine's state and interacts with the elevator as necessary, using {@link Commands} to request modifications to the elevator state.
	 * <br><br>
	 * 
	 * States and behavior:
	 *	<ul>
	 * 		<li>
	 * 			{@link ElevatorCustomPositioningRoutineState#START}: 
	 * 			Requests the elevator to switch to {@link ElevatorState#CUSTOM_POSITIONING} and transitions to the {@code WAITING} state.
	 * 		</li>	
	 * 		<li>
	 * 			{@link ElevatorCustomPositioningRoutineState#WAITING}:
	 * 			Checks if the {@link Elevator} has been calibrated by attempting to retrieve the top and bottom position encoder values.
	 * 			If so, set the startTime to the current system time, set the encoder position target using {@link Elevator#setWantedPosition(double)}, and transition
	 * 			to the {@code MOVING} state.
	 * 		</li>	
	 * 		<li>
	 * 			{@link ElevatorCustomPositioningRoutineState#MOVING}:
	 * 			At this point, the elevator should be moving using a built-in talon control loop. All the routine needs to do is check if it's done under one
	 * 			of two conditions: <br>
	 * 			1. The elevator has reached its target and is in the {@link ElevatorState#HOLD} state. <br>
	 * 			2. The routine has timed out. <br>
	 * 			When one of these conditions is met, the routine transitions to the {@code DONE} state.
	 * 		</li>	
	 * 		<li>
	 * 			{@link ElevatorCustomPositioningRoutineState#DONE}
	 * 			Ensures that the elevator is not moving by requesting the {@link ElevatorState#IDLE} state.
	 * 		</li>
	 * 	</ul>
	 * 
	 * @param commands
	 *            {@link Commands}
	 */
	@Override
	public Commands update(Commands commands) {
		switch(mState) {
			case START:
				commands.wantedElevatorState = Elevator.ElevatorState.CUSTOM_POSITIONING;
				//Don't start immediately: wait until elevator is calibrated, because it might not be right now
				mState = ElevatorCustomPositioningRoutineState.WAITING;
				break;
			case WAITING:
				if (elevator.getElevatorBottomPosition().isPresent() && elevator.getElevatorBottomPosition().isPresent()) {
					//Routine effectively begins now
					
					double bottom = elevator.getElevatorBottomPosition().get(), top = elevator.getElevatorTopPosition().get();
					//Set target encoder value
					switch(mPosition) {
						case TOP:
							elevator.setWantedPosition(top);
							break;
						case SWITCH:
							//We estimate the switch position to be a certain percentage of the way from bottom to top
							elevator.setWantedPosition(bottom + (top - bottom) * Constants.kSwitchPositionPercentage);
							break;
						case BOTTOM:
							elevator.setWantedPosition(bottom);
							break;
					}
					mState = ElevatorCustomPositioningRoutineState.MOVING;
					mStartTime = System.currentTimeMillis();
				}
				break;
			case MOVING:
				//Take advantage of the fact that the elevator holds after it is in position
				if(elevator.getState() == Elevator.ElevatorState.HOLD) {
					mState = ElevatorCustomPositioningRoutineState.DONE;
				}
				//Terminate on timeout
				if(System.currentTimeMillis() - mStartTime > mTimeout * 1000) {
					Logger.getInstance().logRobotThread(Level.WARNING, "Elevator custom positioning routine timed out!");
					mState = ElevatorCustomPositioningRoutineState.DONE;
				}
				break;
			case DONE:
				commands.wantedElevatorState = ElevatorState.IDLE;
				break;
		}
		return commands;
	}

	@Override
	public Commands cancel(Commands commands) {
		mState = ElevatorCustomPositioningRoutineState.DONE;
		return commands;
	}

	@Override
	public boolean finished() {
		return mState == ElevatorCustomPositioningRoutineState.DONE;
	}

	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[] { elevator };
	}

	@Override
	public String getName() {
		return "ElevatorCustomPositioningRoutine";
	}

}
