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

    public RightStartRightScaleAutoMode(Alliance alliance) {
        super(alliance);
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
        DrivePathRoutine toScaleDrivePath = getToScale();

        ArrayList<Routine> prepareElevatorIntakeToScale = new ArrayList<>();
        prepareElevatorIntakeToScale.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches+16, 1.6));
        prepareElevatorIntakeToScale.add(new SequentialRoutine(new IntakeDownRoutine(), new IntakeCloseRoutine()));

        ParallelRoutine toScale = new ParallelRoutine(toScaleDrivePath, new SequentialRoutine(
                new IntakeCloseRoutine(), new IntakeDownRoutine(), new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches + 16, 1.6),
                new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches,
                1.6), toScaleDrivePath, "p1")
        ));

        ParallelRoutine dropAndReset = new ParallelRoutine(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1),
                new DriveSensorResetRoutine(1.0));

        DrivePathRoutine backUpPath = getBackward();
        ParallelRoutine backUp = new ParallelRoutine(backUpPath, new WaypointTriggerRoutine(
                new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,3), backUpPath, "p5"));

        return new SequentialRoutine(new DriveSensorResetRoutine(1.0), toScale, dropAndReset, backUp, turnIntake());
    }

    public Routine turnIntake() {
        ArrayList<Routine> backupIntake = new ArrayList<Routine>();
        backupIntake.add(new CascadingGyroEncoderTurnAngleRoutine(120));


        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 45.0));
        path.add(new Path.Waypoint(new Translation2d(60, 25), 0));

        backupIntake.add(new ParallelRoutine(new DrivePathRoutine(new Path(path), false),
                         new IntakeSensorStopRoutine(Intake.WheelState.INTAKING, 2)));

        return new SequentialRoutine(backupIntake);

    }

    public DrivePathRoutine getToScale() {
        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 120.0));
        if(mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightSwitchX + Constants.kRobotLengthInches,
                    0.0), 100, "p1"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches + AutoDistances.kBlueRightCornerOffset)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 70, "p2"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueRightCornerOffset)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0, "p3"));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightSwitchX + Constants.kRobotLengthInches,
                    0.0), 100, "p1"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches + AutoDistances.kRedRightCornerOffset)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 70, "p2"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kRedRightCornerOffset)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0, "p3"));
        }

        return new DrivePathRoutine(new Path(path), false);

    }

    public DrivePathRoutine getBackward() {

        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 45.0, "p4"));
        path.add(new Path.Waypoint(new Translation2d(-20.0, 0.0), 30, "p5"));
        path.add(new Path.Waypoint(new Translation2d(-30.0, 0.0), 0, "p6"));

        return new DrivePathRoutine(new Path(path), true);
    }

    @Override
    public String getKey() {
        return mAlliance + " RIGHT SCALE RIGHT";
    }
}