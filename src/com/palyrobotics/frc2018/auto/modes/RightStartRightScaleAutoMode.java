package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.WaypointTriggerRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.*;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.*;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RightStartRightScaleAutoMode extends AutoModeBase {

    @Override
    public String toString() {
        return mAlliance + this.getClass().toString();
    }

    @Override
    public void prestart() {

    }

    @Override
    public Routine getRoutine() {
        DrivePathRoutine toScaleDrivePath = getToScale();

        ArrayList<Routine> prepareElevatorIntakeToScale = new ArrayList<>();
        prepareElevatorIntakeToScale.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches+16, .75));
        prepareElevatorIntakeToScale.add(new SequentialRoutine(new IntakeDownRoutine(), new IntakeCloseRoutine()));

        ParallelRoutine toScale = new ParallelRoutine(toScaleDrivePath, new SequentialRoutine(
                new IntakeCloseRoutine(), new IntakeDownRoutine(), new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches + 16, 1.6),
                new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches,
                1.6), toScaleDrivePath, "p1")
        ));

        ParallelRoutine dropAndReset = new ParallelRoutine(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, .8));

        DrivePathRoutine backUpPath = getBackward();
        ParallelRoutine backUp = new ParallelRoutine(backUpPath, new WaypointTriggerRoutine(
                new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,1.05), backUpPath, "p5"));

        return new SequentialRoutine(new DriveSensorResetRoutine(1.0), toScale, new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1), backUp);
    }

    public Routine turnIntake() {
        ArrayList<Routine> backupIntake = new ArrayList<Routine>();

//        List<Path.Waypoint> path = new ArrayList<>();

//        path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches- Constants.kNullZoneAllowableBack-55,
//                -(Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueRightCornerOffset)
//                        + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.5-55), 50, "p5"));
//        path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightSwitchX + AutoDistances.kSwitchPlateLength,
//                -(AutoDistances.kBlueRightCornerOffset)+AutoDistances.kBlueRightSwitchX), 0));
//
//        backupIntake.add(new DrivePathRoutine(new Path(path), false));

        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 55));
        path.add(new Path.Waypoint(new Translation2d(60, 0), 0));

        List<Path.Waypoint> backup = new ArrayList<>();

        backup.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 60));
        backup.add(new Path.Waypoint(new Translation2d(-8, 0), 0));

        List<Path.Waypoint> forward = new ArrayList<>();

        forward.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 60));
        forward.add(new Path.Waypoint(new Translation2d(9, 0), 0));

        backupIntake.add(new CascadingGyroEncoderTurnAngleRoutine(-90));
        backupIntake.add(new ParallelRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,.3),
                new DriveUntilHasCubeRoutine(new DrivePathRoutine(new Path(path), false),1.2)));
        backupIntake.add(new DriveSensorResetRoutine(.01));
        backupIntake.add(new DrivePathRoutine(new Path(backup), true));
        backupIntake.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 1));
        backupIntake.add(new DrivePathRoutine(new Path(forward), false));
        backupIntake.add(new IntakeWheelRoutine(Intake.WheelState.EXPELLING,.4));

        return new SequentialRoutine(backupIntake);

    }

    public DrivePathRoutine getToScale() {
        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 120.0));
        if(mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightSwitchX + Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches + AutoDistances.kBlueRightCornerOffset)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/9), 100, "p1"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches + AutoDistances.kBlueRightCornerOffset)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/5), 58, "p2"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches- Constants.kNullZoneAllowableBack,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueRightCornerOffset)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/5), 0.0, "p3"));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightSwitchX + Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches + AutoDistances.kRedRightCornerOffset)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/9), 100, "p1"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches + AutoDistances.kRedRightCornerOffset)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/5), 58, "p2"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches- Constants.kNullZoneAllowableBack,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kRedRightCornerOffset)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/5), 0.0, "p3"));
        }

        return new DrivePathRoutine(new Path(path), false);

    }

    public DrivePathRoutine getBackward() {

        ArrayList<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 45.0, "p4", true));
        path.add(new Path.Waypoint(new Translation2d(-15.0, -15.0), 30, "p5", true));
        path.add(new Path.Waypoint(new Translation2d(-30.0, -30.0), 0, "p6", true));

        return new DrivePathRoutine(path, true, true);
    }


    @Override
    public String getKey() {
        return mAlliance + " RIGHT SCALE RIGHT";
    }
}