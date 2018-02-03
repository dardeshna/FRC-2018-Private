package com.palyrobotics.frc2018.behavior.routines.elevator;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Elevator.ElevatorState;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.util.logger.Logger;

import java.util.Optional;
import java.util.logging.Level;

public class ElevatorCustomPositioningRoutine extends Routine {

	//Which elevator position should move to
	private double mPosition;
	
	//How long the routine has until it times out (seconds)
	private double mTimeout;
	
	private long mStartTime;

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
	
	@Override
	public void start() {
		mStartTime = System.currentTimeMillis();
	}

	/**
	 * Sets the requested elevator setpoint through the Commands object.
	 * @param commands
	 *            {@link Commands}
	 */
	@Override
	public Commands update(Commands commands) {
		commands.wantedElevatorState = Elevator.ElevatorState.CUSTOM_POSITIONING;
		commands.robotSetpoints.elevatorPositionSetpoint = Optional.of(mPosition);
		return commands;
	}

	@Override
	public Commands cancel(Commands commands) {
		commands.wantedElevatorState = Elevator.ElevatorState.HOLD;
		return commands;
	}

	@Override
	public boolean finished() {
		//Terminate upon timeout
		if(System.currentTimeMillis() - mStartTime > mTimeout * 1000) {
			Logger.getInstance().logRobotThread(Level.WARNING, "Elevator custom positioning routine timed out!");
			return true;
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
