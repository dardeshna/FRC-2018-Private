package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.GyroMotionMagicTurnAngleRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
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
        path.add(new Waypoint(new Translation2d(0,0), 150));
        double distanceX = mDistances.kLeftScaleX - Constants.kRobotLengthInches + mDistances.kScalePlateLength/2;

        path.add(new Waypoint(new Translation2d(distanceX, 0.0), 0.0));

        ArrayList<Routine> routines = new ArrayList<>();
        routines.add(new DriveSensorResetRoutine(.1));
        routines.add(new SequentialRoutine(new DrivePathRoutine(new Path(path), false), new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 1.0)));
        routines.add(new GyroMotionMagicTurnAngleRoutine(90));
        routines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 4));
        routines.add(new IntakeWheelRoutine(Intake.WheelState.VAULT_EXPELLING, .3));

        return new SequentialRoutine(routines);
    }

    @Override
    public String getKey() {
        return "This auto is deployed in a different way.";
    }
}
