package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.WaypointTriggerRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeCloseRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RightStartLeftScaleAutoMode extends AutoModeBase {

    public RightStartLeftScaleAutoMode(Alliance alliance) {
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
        toScaleSubsystems.add(new IntakeDownRoutine());
        toScaleSubsystems.add(new IntakeCloseRoutine());
        toScaleSubsystems.add(new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches,
                1.6), toScaleDrivePath, "p4"));
        ParallelRoutine toScale = new ParallelRoutine(toScaleDrivePath, new ParallelRoutine(toScaleSubsystems));

        ParallelRoutine dropAndReset = new ParallelRoutine(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1),
                new DriveSensorResetRoutine(1.0));

        DrivePathRoutine backUpPath = getBackward();
        ArrayList<Routine> backUpSubsystems = new ArrayList<>();
        backUpSubsystems.add(new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,
                3), backUpPath, "p8"));
        ParallelRoutine backUp = new ParallelRoutine(backUpPath, new ParallelRoutine(backUpSubsystems));

        return new SequentialRoutine(new DriveSensorResetRoutine(1.0), toScale, dropAndReset, backUp);
    }

    public DrivePathRoutine getToScale() {
        List<Waypoint> path = new ArrayList<>();

        path.add(new Waypoint(new Translation2d(0.0, 0.0), 140));
        if(mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d((AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0)/2,
                    -Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueRightCornerOffset + AutoDistances.kBlueRightSwitchY/2.0), 80, "p1"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueRightCornerOffset + AutoDistances.kBlueRightSwitchY/2.0), 67.5, "p2"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueRightCornerOffset + AutoDistances.kBlueRightSwitchY/2.0+20), 80, "p3"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    (AutoDistances.kFieldWidth - Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueRightCornerOffset
                            - AutoDistances.kBlueLeftScaleY - AutoDistances.kScalePlateWidth/2.0
                            -Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueRightCornerOffset + AutoDistances.kBlueRightSwitchY/2.0)/2), 70.0, "p4"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    AutoDistances.kFieldWidth - Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueRightCornerOffset
                            - AutoDistances.kBlueLeftScaleY - AutoDistances.kScalePlateWidth/2.0-20), 30.0, "p5"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueLeftScaleX - Constants.kRobotLengthInches-Constants.kScaleOffset,
                    AutoDistances.kFieldWidth - Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueRightCornerOffset
                            - AutoDistances.kBlueLeftScaleY - AutoDistances.kScalePlateWidth/2.0-25), 0.0, "p6"));
        } else {
            path.add(new Path.Waypoint(new Translation2d((AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0)/2,
                    -Constants.kRobotWidthInches/2.0 - AutoDistances.kRedRightCornerOffset + AutoDistances.kRedRightSwitchY/2.0), 80, "p1"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -Constants.kRobotWidthInches/2.0 - AutoDistances.kRedRightCornerOffset + AutoDistances.kRedRightSwitchY/2.0), 67.5, "p2"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -Constants.kRobotWidthInches/2.0 - AutoDistances.kRedRightCornerOffset + AutoDistances.kRedRightSwitchY/2.0+20), 67.5, "p3"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    (AutoDistances.kFieldWidth - Constants.kRobotWidthInches/2.0 - AutoDistances.kRedRightCornerOffset
                            - AutoDistances.kRedLeftScaleY - AutoDistances.kScalePlateWidth/2.0
                            -Constants.kRobotWidthInches/2.0 - AutoDistances.kRedRightCornerOffset
                            + AutoDistances.kRedRightSwitchY/2.0)/2), 70.0, "p4"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    AutoDistances.kFieldWidth - Constants.kRobotWidthInches/2.0 - AutoDistances.kRedRightCornerOffset
                            - AutoDistances.kRedLeftScaleY - AutoDistances.kScalePlateWidth/2.0), 30.0, "p5"));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedLeftScaleX - Constants.kRobotLengthInches-Constants.kScaleOffset,
                    AutoDistances.kFieldWidth - Constants.kRobotWidthInches/2.0 - AutoDistances.kRedRightCornerOffset
                            - AutoDistances.kRedLeftScaleY - AutoDistances.kScalePlateWidth/2.0), 0.0, "p6"));
        }

        return new DrivePathRoutine(new Path(path), false);
    }

    public DrivePathRoutine getBackward() {
        List<Path.Waypoint> path = new ArrayList<>();
        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 45.0, "p7", true));
        path.add(new Path.Waypoint(new Translation2d(-30, 0.0), 0, "p8", true));

        return new DrivePathRoutine(new Path(path), true);
    }

	@Override
	public String getKey() {
		return mAlliance + " RIGHT SCALE LEFT";
	}
}
