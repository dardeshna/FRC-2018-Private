package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.TalonSRXRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RightStartRightScaleAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public RightStartRightScaleAutoMode(Alliance alliance) {
        this.mAlliance = alliance;
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
        List<Path.Waypoint> path = new ArrayList<>();
        List<Path.Waypoint> lastSegment = new ArrayList<>();
        
        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 72.0));
        if(mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightSwitchX + Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueRightCornerOffset) + AutoDistances.kBlueRightSwitchY/2.0), 72.0));
            lastSegment.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueRightCornerOffset)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 55));
            lastSegment.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches/2,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueRightCornerOffset)
                    + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightSwitchX + Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kRedRightCornerOffset) + AutoDistances.kRedRightSwitchY/2.0), 72.0));
            lastSegment.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kRedRightCornerOffset)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 55));
            lastSegment.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches/2,
                    -(Constants.kRobotWidthInches/2.0 + AutoDistances.kRedRightCornerOffset)
                    + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0));
        }
        
        ArrayList<Routine> routines = new ArrayList<Routine>();

        //Reset sensors before
        routines.add(new DriveSensorResetRoutine());

        //Drive path while moving elevator up and moving intake down
        ArrayList<Routine> inTransitRoutines = new ArrayList<>();
        inTransitRoutines.add(new DrivePathRoutine(new Path(path), false));
        inTransitRoutines.add(new IntakeDownRoutine());
        inTransitRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches - 8, 2.6));
        routines.add(new ParallelRoutine(inTransitRoutines));

        ArrayList<Routine> lastSegmentElevator = new ArrayList<>();
        lastSegmentElevator.add(new DrivePathRoutine(new Path(lastSegment), false));
        lastSegmentElevator.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 2.6));
        routines.add(new ParallelRoutine(lastSegmentElevator));
        routines.add(new DriveSensorResetRoutine());

        ArrayList<Routine> parallelDrop = new ArrayList<>();
        parallelDrop.add(getForward());
        //Open when everything is done to score
        parallelDrop.add(new IntakeWheelRoutine(Intake.WheelState.EXPELLING,1));
        routines.add(new ParallelRoutine(parallelDrop));
        routines.add(new DriveSensorResetRoutine());
        routines.add(getBackward());
        routines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches,3));

        return new SequentialRoutine(routines);
    }

    public Routine getForward() {
        ArrayList<Routine> backUp = new ArrayList<>();

        //zero drive sensors
        backUp.add(new DriveSensorResetRoutine());

        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 72.0));
        path.add(new Path.Waypoint(new Translation2d(9.0, 0.0), 0));
        backUp.add(new DrivePathRoutine(new Path(path), false));

        return new SequentialRoutine(backUp);
    }

    public Routine getBackward() {
        ArrayList<Routine> backUp = new ArrayList<>();

        //zero drive sensors
        backUp.add(new DriveSensorResetRoutine());

        List<Path.Waypoint> path = new ArrayList<>();

        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 72.0));
        path.add(new Path.Waypoint(new Translation2d(-8, 0.0), 0));
        backUp.add(new DrivePathRoutine(new Path(path), true));

        return new SequentialRoutine(backUp);
    }

	@Override
	public String getKey() {
		return mAlliance + " RIGHT SCALE RIGHT";
	}
}
