package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.auto.AutoModeBase.Alliance;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;
import com.palyrobotics.frc2018.util.trajectory.Path;

import java.util.ArrayList;
import java.util.List;

public class CenterStartLeftMultiSwitchAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public CenterStartLeftMultiSwitchAutoMode(AutoModeBase.Alliance alliance) {
        this.mAlliance = alliance;
    }


    @Override
    public String toString() {
        return mAlliance + this.getClass().toString();
    }

    @Override
    public void prestart() {

    }

    @Override
    public Routine getRoutine() {
        ArrayList<Routine> routines = new ArrayList<>();
        routines.add(new CenterStartLeftSwitchAutoMode(this.mAlliance).getRoutine());
        return new SequentialRoutine(routines);
    }

    public ParallelRoutine getExpelDrop() {
        return new ParallelRoutine(new ArrayList<Routine>() {{
            new IntakeOpenRoutine();
            new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1);
        }});
    }

    public DrivePathRoutine getDriveForward() {
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        path.add(new Waypoint(new Translation2d(12,0), 0));
        return new DrivePathRoutine(new Path(path), false);
    }


    public DrivePathRoutine getDriveBack() {
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        path.add(new Waypoint(new Translation2d(12,0), 0));
        return new DrivePathRoutine(new Path(path), false);
    }

    @Override
    public String getKey() {
        return mAlliance + " Center Switch Left";
    }

}
