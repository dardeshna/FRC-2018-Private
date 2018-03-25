package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class LeftStartLeftMultiScaleAutoMode extends AutoModeBase {

    public LeftStartLeftMultiScaleAutoMode(Alliance alliance) {
        super(alliance);
    }

    @Override
    public String toString() {
        return "Left Multi Scale Auto Mode";
    }

    @Override
    public void prestart() {

    }

    public DrivePathRoutine getPathToCubeOrBack() {
        List<Path.Waypoint> path = new ArrayList<>();
        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 72.0));
        path.add(new Path.Waypoint(new Translation2d(107,30),0));

        return new DrivePathRoutine(new Path(path), false);
    }


    @Override
    public Routine getRoutine() {
        ArrayList<Routine> routines = new ArrayList<>();

        routines.add(new LeftStartLeftScaleAutoMode(this.mAlliance).getRoutine());

    //        routines.add(new CascadingTurnAngle(Math.PI));
        routines.add(new ParallelRoutine(new ArrayList<Routine>() {{
            getPathToCubeOrBack();
            new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches, 5);
        }}));
//        routines.add(new IntakeUntilHasCubeRoutine());
//        routines.add(new CascadingTurnAngle(Math.PI));
        routines.add(new ParallelRoutine(new ArrayList<Routine>() {{
            getPathToCubeOrBack();
            new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 5);
        }}));

        routines.add(new IntakeOpenRoutine());

        return new SequentialRoutine(routines);
    }

    @Override
    public String getKey() {
        return mAlliance + " LEFT SCALE LEFT SCALE LEFT";
    }
}
