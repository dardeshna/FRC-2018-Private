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

public class LeftStartLeftScaleLeftSwitchAutoMode extends AutoModeBase{

    @Override
    public String toString() {
        return mAlliance + this.getClass().toString();
    }

    @Override
    public void prestart() {

    }

    @Override
    public Routine getRoutine() {

        DriveUntilHasCubeRoutine getCube = new DriveUntilHasCubeRoutine(new DrivePathRoutine(getDriveToCube(), false, true), 3.0);

        return new SequentialRoutine(new LeftStartLeftScaleAutoMode().getRoutine(), getCube, new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 0.7),
                new DrivePathRoutine(getDriveIntoSwitch(), false, true), new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1.0));
    }

    public ArrayList<Path.Waypoint> getDriveIntoSwitch() {
        ArrayList<Path.Waypoint> path = new ArrayList<>();
        path.add(new Path.Waypoint(new Translation2d(0, 0), 40.0, true));
        path.add(new Path.Waypoint(new Translation2d(-10.0, -3.5), 0.0, true));
        return path;
    }

    public ArrayList<Path.Waypoint> getDriveToCube() {
        ArrayList<Path.Waypoint> path = new ArrayList<>();
        path.add(new Path.Waypoint(new Translation2d(0, 0), 40.0, true));
        if(mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueLeftSwitchX + AutoDistances.kSwitchPlateLength + Constants.kSquareCubeLength - Constants.kRobotLengthInches + Constants.kCenterOfRotationOffsetFromFrontInches,
                    AutoDistances.kBlueLeftCornerOffset - AutoDistances.kBlueLeftSwitchY + Constants.kRobotWidthInches/2.0), 0.0, false));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedLeftSwitchX + AutoDistances.kSwitchPlateLength + Constants.kSquareCubeLength - Constants.kRobotLengthInches + Constants.kCenterOfRotationOffsetFromFrontInches,
                    AutoDistances.kRedLeftCornerOffset - AutoDistances.kRedLeftSwitchY + Constants.kRobotWidthInches/2.0), 0.0, false));
        }

        return path;
    }

    @Override
    public String getKey() {
        return mAlliance + " LEFT SCALE LEFT SWITCH LEFT";
    }
}
