package com.palyrobotics.frc2018.behavior.routines.elevator;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Subsystem;

/**
 * @author Jason
 */

public class ElevatorIdleRoutine extends Routine {

    private boolean alreadyRan;

    @Override
    public void start() {
        alreadyRan = false;
    }

    @Override
    public Commands update(Commands commands) {
        commands.wantedElevatorState = Elevator.ElevatorState.IDLE;
        alreadyRan = true;
        return commands;
    }

    @Override
    public Commands cancel(Commands commands) {
        return commands;
    }

    @Override
    public boolean finished() {
        return alreadyRan;
    }

    @Override
    public Subsystem[] getRequiredSubsystems() {
        return new Subsystem[] { elevator };
    }

    @Override
    public String getName() {
        return "ElevatorIdleRoutine";
    }
}
