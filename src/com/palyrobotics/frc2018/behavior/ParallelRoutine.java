package com.palyrobotics.frc2018.behavior;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Nihar on 12/27/16.
 */
public class ParallelRoutine extends Routine {
	private ArrayList<Routine> mRoutines;

	/**
	 * Runs all routines at the same time Finishes when all routines finish
	 * 
	 * @param routines
	 */
	public ParallelRoutine(ArrayList<Routine> routines) {
		this.mRoutines = routines;
	}

    public ParallelRoutine(Routine... routines) {
	    mRoutines = new ArrayList<>(Arrays.asList(routines));
    }

    @Override
	public void start() {
		for(Routine routine : mRoutines) {
			routine.start();
		}
	}

	@Override
	public Commands update(Commands commands) {
		for(Routine routine : mRoutines) {
			if(!routine.finished()) {
				commands = routine.update(commands);
			}
		}
		return commands;
	}

	@Override
	public Commands cancel(Commands commands) {
		for(Routine routine : mRoutines) {
			commands = routine.cancel(commands);
		}
		return commands;
	}

	@Override
	public boolean finished() {
		for(Routine routine : mRoutines) {
			if(!routine.finished()) {
				return false;
			}
		}
		return true;
	}

	@Override
	public Subsystem[] getRequiredSubsystems() {
		return RoutineManager.sharedSubsystems(mRoutines);
	}


	@Override
	public ArrayList<Routine> getEnclosingSequentialRoutine() {
		return this.mRoutines;
	}

	@Override
	public String getName() {
		String name = "ParallelRoutine of (";
		for(Routine routine : mRoutines) {
			name += (routine.getName() + " ");
		}
		return name + ")";
	}
}
