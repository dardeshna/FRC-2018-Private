package com.palyrobotics.frc2018.behavior.routines.elevator;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.trajectory.Path;

import java.util.Optional;
import java.util.logging.Level;

public class ElevatorCustomPositioningRoutine extends Routine {

	//Which elevator position should move to
	private double mPosition;

	//How long the routine has until it times out (seconds)
	private double mTimeout;

	private long mStartTime = -1;

	//If run in auto, pass it in the path and when you want to start raising the elevator
	private Optional<Path> mPath = Optional.empty();

	private Optional<String> mRoutineStartWayPoint = Optional.empty();

	/**
	 * Constructor. Moves the elevator to a desired position and holds it afterwards.
	 *
	 * @param position from the bottom of the elevator, in inches
	 * @param timeout time before routine breaks out, in seconds
	 */
	public ElevatorCustomPositioningRoutine(double position, double timeout) {
		mPosition = position;
		mTimeout = timeout;
	}

	/**
	 * Constructor which should only be used in auto. Moves the elevator to a desired position and holds it afterwards.
	 * Only begins the routine after the robot has reached a certain point on the auto path.
	 *
	 * @param position from the bottom of the elevator, in inches
	 * @param timeout time before routine breaks out, in seconds
	 * @param path path that is being driven in auto
	 * @param routineStartWaypoint which waypoint the routine should begin at
	 */
	public ElevatorCustomPositioningRoutine(double position, double timeout, Path path, String routineStartWaypoint) {
		mPosition = position;
		mTimeout = timeout;
		mPath = Optional.of(path);
		mRoutineStartWayPoint = Optional.of(routineStartWaypoint);
	}

	@Override
	public void start() {
	    if(!mPath.isPresent())
		mStartTime = System.currentTimeMillis();
	}

	/**
	 * Sets the requested elevator setpoint through the Commands object.
	 * @param commands
	 *            {@link Commands}
	 */
	@Override
	public Commands update(Commands commands) {
	    if(mPath.isPresent()) {
//            System.out.println("Markers crossed: " + mPath.get().getMarkersCrossed());
        }
        if(mPath.isPresent() && mPath.get().getMarkersCrossed().contains(mRoutineStartWayPoint.get())) {
//            System.out.println("in right case   ");
        }
		if(!mPath.isPresent() || mRoutineStartWayPoint.isPresent() && mPath.get().getMarkersCrossed().contains(mRoutineStartWayPoint.get())) {
	        if(mStartTime == -1) mStartTime = System.currentTimeMillis();
			commands.wantedElevatorState = Elevator.ElevatorState.CUSTOM_POSITIONING;
			commands.robotSetpoints.elevatorPositionSetpoint = Optional.of(mPosition);
		}
		return commands;
	}

	@Override
	public Commands cancel(Commands commands) {
		commands.wantedElevatorState = Elevator.ElevatorState.HOLD;
		return commands;
	}

	@Override
	public boolean finished() {
		//Terminate upon
        if(mStartTime != -1) {
            if (System.currentTimeMillis() - mStartTime > mTimeout * 1000) {
//                Logger.getInstance().logRobotThread(Level.WARNING, "Elevator custom positioning routine timed out!");
                return true;
            }
        }

		if(elevator.getElevatorBottomPosition().isPresent() && elevator.getElevatorWantedPosition().isPresent()) {
			if(elevator.getElevatorWantedPosition().get() == elevator.getElevatorBottomPosition().get() && robotState.elevatorBottomHFX) {
				Logger.getInstance().logRobotThread(Level.INFO, "Elevator custom positioning routine finished due to bottom HFX");
				return true;
			}
		}

		return elevator.onTarget();
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