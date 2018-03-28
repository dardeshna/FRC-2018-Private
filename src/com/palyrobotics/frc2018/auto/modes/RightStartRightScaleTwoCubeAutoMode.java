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

public class RightStartRightScaleTwoCubeAutoMode extends AutoModeBase {

    public RightStartRightScaleTwoCubeAutoMode(Alliance alliance) {
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
        ArrayList<Routine> toScaleSubsystems = new ArrayList<>();
        toScaleSubsystems.add(new DriveSensorResetRoutine(.1));
        toScaleSubsystems.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches+16, 1.6));
        toScaleSubsystems.add(new IntakeDownRoutine());
        toScaleSubsystems.add(new IntakeCloseRoutine());
        toScaleSubsystems.add(new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches,
                1.6), toScaleDrivePath, "p1"));
        ParallelRoutine toScale = new ParallelRoutine(toScaleDrivePath, new ParallelRoutine(toScaleSubsystems));

        ParallelRoutine dropAndReset = new ParallelRoutine(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1),
                new DriveSensorResetRoutine(1.0));

        DrivePathRoutine backUpPath = getBackward();
        ParallelRoutine backUp = new ParallelRoutine(backUpPath, new WaypointTriggerRoutine(
                new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,3), backUpPath, "p5"));

        return new SequentialRoutine(new DriveSensorResetRoutine(1.0), toScale, dropAndReset, backUp, turnIntake());
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

    public Routine turnIntake() {
        ArrayList<Routine> backupIntake = new ArrayList<Routine>();
        backupIntake.add(new CascadingGyroEncoderTurnAngleRoutine(120));
        backupIntake.add(new DriveSensorResetRoutine(.1));

        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 45.0, true));
        path.add(new Path.Waypoint(new Translation2d(60, 25), 0, true));

        backupIntake.add(new ParallelRoutine(new DriveUntilHasCubeRoutine(new DrivePathRoutine(new Path(path), false))));

        return new SequentialRoutine(backupIntake);

    }


    public DrivePathRoutine getBackward() {

        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 45.0, "p4", true));
        path.add(new Path.Waypoint(new Translation2d(-30, 0.0), 0, "p5", true));

        return new DrivePathRoutine(new Path(path), true);
    }

    @Override
    public String getKey() {
        return mAlliance + " RIGHT SCALE RIGHT";
    }
}