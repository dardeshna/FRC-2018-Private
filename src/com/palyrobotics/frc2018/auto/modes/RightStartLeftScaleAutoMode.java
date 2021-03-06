package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.WaypointTriggerRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
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
//        toScaleSubsystems.add(new IntakeDownRoutine());
//        toScaleSubsystems.add(new IntakeCloseRoutine());
        toScaleSubsystems.add(new SequentialRoutine(new IntakeCloseRoutine(), new IntakeDownRoutine(), new TimeoutRoutine(Constants.kScaleAutoWaitBeforeElevatorRaiseTimeSeconds), new ElevatorCustomPositioningRoutine(Constants.kElevatorCubeInTransitPositionInches, 1.0)));
        toScaleSubsystems.add(new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches,
                1.6), toScaleDrivePath, "p4"));
        ParallelRoutine toScale = new ParallelRoutine(toScaleDrivePath, new SequentialRoutine(toScaleSubsystems));

        DrivePathRoutine backUpPath = getBackward();
        ArrayList<Routine> backUpSubsystems = new ArrayList<>();
        backUpSubsystems.add(new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,
                3), backUpPath, "p8"));
        ParallelRoutine backUp = new ParallelRoutine(backUpPath, new ParallelRoutine(backUpSubsystems));

        return new SequentialRoutine(new DriveSensorResetRoutine(0.75), new TimeoutRoutine(5.5), toScale, new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 0.75), backUp);
    }

    public DrivePathRoutine getToScale() {
        List<Waypoint> path = new ArrayList<>();

        path.add(new Waypoint(new Translation2d(0.0, 0.0), 140));

        path.add(new Path.Waypoint(new Translation2d((mDistances.kScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0)/2,
                -Constants.kRobotWidthInches/2.0 - mDistances.kRightCornerOffset + mDistances.kRightSwitchY/2.0), 80, "p1"));
        path.add(new Path.Waypoint(new Translation2d(mDistances.kScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                -Constants.kRobotWidthInches/2.0 - mDistances.kRightCornerOffset + mDistances.kRightSwitchY/2.0), 67.5, "p2"));
        path.add(new Path.Waypoint(new Translation2d(mDistances.kScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                -Constants.kRobotWidthInches/2.0 - mDistances.kRightCornerOffset + mDistances.kRightSwitchY/2.0+20), 80, "p3"));
        path.add(new Path.Waypoint(new Translation2d(mDistances.kScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                (mDistances.kFieldWidth - Constants.kRobotWidthInches/2.0 - mDistances.kRightCornerOffset
                        - mDistances.kLeftScaleY - mDistances.kScalePlateWidth/2.0
                        -Constants.kRobotWidthInches/2.0 - mDistances.kRightCornerOffset + mDistances.kRightSwitchY/2.0)/2), 70.0, "p4"));
        path.add(new Path.Waypoint(new Translation2d(mDistances.kScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                mDistances.kFieldWidth - Constants.kRobotWidthInches/2 - mDistances.kRightCornerOffset
                        - mDistances.kLeftScaleY), 30.0, "p5"));
        path.add(new Path.Waypoint(new Translation2d(mDistances.kLeftScaleX - Constants.kRobotLengthInches-Constants.kNullZoneAllowableBack,
                mDistances.kFieldWidth - Constants.kRobotWidthInches/2 - mDistances.kRightCornerOffset
                        - mDistances.kLeftScaleY - mDistances.kScalePlateWidth/5.0), 0.0, "p6"));
        

        return new DrivePathRoutine(new Path(path), false);
    }

    public DrivePathRoutine getBackward() {
        ArrayList<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 50.0, "p7", true));
        path.add(new Path.Waypoint(new Translation2d(-15.0, 15.0), 40.0, "p8", true));
        path.add(new Path.Waypoint(new Translation2d(-30.0, 30.0), 0, "p9", true));

        return new DrivePathRoutine(path, true, true);
    }

	@Override
	public String getKey() {
		return mAlliance + " RIGHT SCALE LEFT";
	}
}
