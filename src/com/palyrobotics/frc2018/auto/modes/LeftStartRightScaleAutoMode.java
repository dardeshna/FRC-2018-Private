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
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class LeftStartRightScaleAutoMode extends AutoModeBase {

    public LeftStartRightScaleAutoMode(Alliance alliance) {
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

        return new SequentialRoutine(new DriveSensorResetRoutine(1.0), toScale, dropAndReset, backUp);
    }

    public DrivePathRoutine getToScale() {
        List<Path.Waypoint> path = new ArrayList<>();
        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 72.0));
        if(mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset - AutoDistances.kBlueLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset - AutoDistances.kRedLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0));
        }
        return new DrivePathRoutine(new Path(path), false);
    }

    public DrivePathRoutine getBackward() {

        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 45.0, "p4", true));
        path.add(new Path.Waypoint(new Translation2d(-20.0, 0.0), 30, "p5", true));
        path.add(new Path.Waypoint(new Translation2d(-30.0, 0.0), 0, "p6", true));

        return new DrivePathRoutine(new Path(path), true);
    }

	@Override
	public String getKey() {
		return mAlliance + " LEFT SCALE RIGHT";
	}
}
