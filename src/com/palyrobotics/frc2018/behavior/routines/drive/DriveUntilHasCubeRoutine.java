package com.palyrobotics.frc2018.behavior.routines.drive;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeCurrentRoutine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.subsystems.Subsystem;

public class DriveUntilHasCubeRoutine extends Routine {

    private DrivePathRoutine drivePathRoutine;
    private IntakeCurrentRoutine intakeRoutine;

    public DriveUntilHasCubeRoutine(DrivePathRoutine drivePathRoutine) {
        this.drivePathRoutine = drivePathRoutine;
        this.intakeRoutine = new IntakeCurrentRoutine();
    }

    @Override
    public void start() {
        drivePathRoutine.start();
        intakeRoutine.start();
    }

    @Override
    public Commands update(Commands commands) {
        drivePathRoutine.update(commands);
        intakeRoutine.update(commands);
        return commands;
    }

    @Override
    public Commands cancel(Commands commands) {
        System.out.println("finished drive until has cube");
        drivePathRoutine.cancel(commands);
        intakeRoutine.cancel(commands);
        return commands;
    }

    @Override
    public boolean finished() {
//        return intakeRoutine.finished();
        return drivePathRoutine.finished();
    }

    @Override
    public Subsystem[] getRequiredSubsystems() {
        return new Subsystem[] {drive, intake};
    }

    @Override
    public String getName() {
        return this.getClass().getName();
    }
}
