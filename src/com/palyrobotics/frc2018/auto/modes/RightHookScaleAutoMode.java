package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.CascadingGyroEncoderTurnAngleRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.GyroMotionMagicTurnAngleRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeCloseRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RightHookScaleAutoMode extends AutoModeBase {

    @Override
    public String toString() {
        return "Right Scale Hook Only Auto";
    }

    @Override
    public void prestart() {

    }

    @Override
    public Routine getRoutine() {
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0,0), 170));
        double distanceX = mDistances.kRightScaleX - Constants.kRobotLengthInches + 1.2 * mDistances.kScalePlateLength/2 - 15;
        double distance2X = mDistances.kRightScaleX - Constants.kRobotLengthInches + 1.4 * mDistances.kScalePlateLength/2;

        path.add(new Waypoint(new Translation2d(distanceX, 0), 20));
        path.add(new Waypoint(new Translation2d(distance2X, 0), 0.0));

        ArrayList<Routine> routines = new ArrayList<>();
        routines.add(new DriveSensorResetRoutine(.1));
        routines.add(new IntakeDownRoutine());
	    routines.add(new IntakeCloseRoutine());
        routines.add(new ParallelRoutine(new DrivePathRoutine(new Path(path), false, 45), new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 1.0)));
        routines.add(new TimeoutRoutine(.5));
        routines.add(new CascadingGyroEncoderTurnAngleRoutine(90));
//        routines.add(new DriveSensorResetRoutine(.1));

//        List<Waypoint> path2 = new ArrayList<>();
//        path2.add(new Waypoint(new Translation2d(0,0), 40));
//        distance2X = -15;
//
//        path.add(new Waypoint(new Translation2d(distance2X, 0), 0));
//
//        routines.add(new DrivePathRoutine(new Path(path2), true));
        routines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 3));
        routines.add(new IntakeWheelRoutine(Intake.WheelState.VAULT_EXPELLING, .3));
        routines.add(new ElevatorCustomPositioningRoutine(5, 3));

        return new SequentialRoutine(routines);
    }

    @Override
    public String getKey() {
        return "This auto is deployed in a different way.";
    }
}
