package com.palyrobotics.frc2018.behavior.routines.intake;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.subsystems.Subsystem;

/**
 * @author Jason
 */
public class IntakeWheelRoutine extends Routine {

    private Intake.WheelState wantedWheelState;
    private boolean alreadyRan;

    public IntakeWheelRoutine(Intake.WheelState wantedWheelState) {
        this.wantedWheelState = wantedWheelState;
    }

    @Override
    public void start() {
        alreadyRan = false;
    }

    @Override
    public Commands update(Commands commands) {
        switch(wantedWheelState) {
            case INTAKING:
                commands.wantedIntakingState = Intake.WheelState.INTAKING;
            case EXPELLING:
                commands.wantedIntakingState = Intake.WheelState.EXPELLING;
            case IDLE:
                commands.wantedIntakingState = Intake.WheelState.IDLE;
        }

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
        return new Subsystem[] { intake };
    }

    @Override
    public String getName() {
        return "IntakeWheelRoutine";
    }
}
