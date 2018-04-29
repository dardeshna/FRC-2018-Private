package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveUntilHasCubeRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;

public class LeftStartRightScaleRightSwitchAutoMode extends AutoModeBase {

    @Override
    public String toString() {
        return mAlliance + this.getClass().toString();
    }

    @Override
    public void prestart() {

    }

    public ArrayList<Path.Waypoint> getDriveIntoSwitch() {
        ArrayList<Path.Waypoint> path = new ArrayList<>();
        path.add(new Path.Waypoint(new Translation2d(0, 0), 40.0, true));
        path.add(new Path.Waypoint(new Translation2d(-10.0, 3.5), 0.0, true));
        return path;
    }

    public ArrayList<Path.Waypoint> getDriveToCube() {
        ArrayList<Path.Waypoint> path = new ArrayList<>();
        path.add(new Path.Waypoint(new Translation2d(0, 0), 40.0, true));

        path.add(new Path.Waypoint(new Translation2d(mDistances.kRightSwitchX + mDistances.kSwitchPlateLength + Constants.kSquareCubeLength - Constants.kRobotLengthInches + Constants.kCenterOfRotationOffsetFromFrontInches,
                -mDistances.kFieldWidth + mDistances.kLeftCornerOffset + Constants.kRobotWidthInches/2.0 + mDistances.kRightSwitchY), 0.0, false));

        return path;
    }

    @Override
    public Routine getRoutine() {
        DriveUntilHasCubeRoutine getCube = new DriveUntilHasCubeRoutine(new DrivePathRoutine(getDriveToCube(), false, true), 3.0);

        return new SequentialRoutine(new LeftStartRightScaleAutoMode().getRoutine(), getCube, new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 0.7),
                new DrivePathRoutine(getDriveIntoSwitch(), false, true), new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1.0));
    }

    @Override
    public String getKey() {
        return mAlliance + " LEFT SCALE RIGHT SWITCH RIGHT";
    }
}
