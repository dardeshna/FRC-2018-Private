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
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class LeftStartLeftScaleAutoMode extends AutoModeBase {

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

//        ArrayList<Routine> prepareElevatorIntakeToScale = new ArrayList<>();
//        prepareElevatorIntakeToScale.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches+16, .75));
//        prepareElevatorIntakeToScale.add(new SequentialRoutine(new IntakeDownRoutine(), new IntakeCloseRoutine()));

        ParallelRoutine toScale = new ParallelRoutine(toScaleDrivePath, new SequentialRoutine(
                new IntakeCloseRoutine(), new IntakeDownRoutine(), new TimeoutRoutine(Constants.kScaleAutoWaitBeforeElevatorRaiseTimeSeconds), new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches + 16, 1.6),
                new WaypointTriggerRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches,
                        1.6), toScaleDrivePath, "p1")
        ));

        DrivePathRoutine backUpPath = getBackward();
        ParallelRoutine backUp = new ParallelRoutine(backUpPath, new WaypointTriggerRoutine(
                new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,3.0), backUpPath, "p5"));

        return new SequentialRoutine(new DriveSensorResetRoutine(0.75), toScale, new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 0.75), backUp);
    }


    public DrivePathRoutine getToScale() {
        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 120.0));

        path.add(new Path.Waypoint(new Translation2d(mDistances.kLeftSwitchX + Constants.kRobotLengthInches,
                (Constants.kRobotWidthInches + mDistances.kLeftCornerOffset)
                        - mDistances.kLeftScaleY - mDistances.kScalePlateWidth/9), 100,"p1"));
        path.add(new Path.Waypoint(new Translation2d(mDistances.kLeftScaleX - 2.0*Constants.kRobotLengthInches,
                (Constants.kRobotWidthInches + mDistances.kLeftCornerOffset)
                        - mDistances.kLeftScaleY - mDistances.kScalePlateWidth/5), 58,"p2"));
        path.add(new Path.Waypoint(new Translation2d(mDistances.kLeftScaleX - Constants.kRobotLengthInches- Constants.kNullZoneAllowableBack,
                (Constants.kRobotWidthInches/2.0 + mDistances.kLeftCornerOffset)
                        - mDistances.kLeftScaleY - mDistances.kScalePlateWidth/5), 0.0, "p3"));
            
        return new DrivePathRoutine(new Path(path), false);
    }

    public DrivePathRoutine getBackward() {

        ArrayList<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 50.0, "p4", true));
//        path.add(new Path.Waypoint(new Translation2d(-15.0, 15.0), 40.0, "p5", true));
//        path.add(new Path.Waypoint(new Translation2d(-30.0, 30.0), 0, "p6", true));
        path.add(new Path.Waypoint(new Translation2d(-15.0, 0.0), 40.0, "p5", true));
        path.add(new Path.Waypoint(new Translation2d(-60.0, 0.0), 0, "p6", true));
        return new DrivePathRoutine(path, true, true);
    }

	@Override
	public String getKey() {
		return mAlliance + " LEFT SCALE LEFT";
	}
}
