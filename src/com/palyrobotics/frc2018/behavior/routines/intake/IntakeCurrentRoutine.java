package com.palyrobotics.frc2018.behavior.routines.intake;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.subsystems.Subsystem;

public class IntakeCurrentRoutine extends Routine {

    public IntakeCurrentRoutine() {

    }

    @Override
    public void start() {

    }

    @Override
    public Commands update(Commands commands) {
        commands.wantedIntakingState = Intake.WheelState.INTAKING;
        commands.wantedIntakeOpenCloseState = Intake.OpenCloseState.NEUTRAL;
        commands.wantedIntakeUpDownState = Intake.UpDownState.DOWN;
        return commands;
    }

    @Override
    public Commands cancel(Commands commands) {
        commands.wantedIntakingState = Intake.WheelState.IDLE;
        commands.wantedIntakeOpenCloseState = Intake.OpenCloseState.CLOSED;
        commands.wantedIntakeUpDownState = Intake.UpDownState.DOWN;
        return commands;
    }

    @Override
    public boolean finished() {
        return robotState.hasCube;
    }

    @Override
    public Subsystem[] getRequiredSubsystems() {
        return new Subsystem[]{ intake };
    }

    @Override
    public String getName() {
        return "IntakeCurrentRoutine";
    }
}
