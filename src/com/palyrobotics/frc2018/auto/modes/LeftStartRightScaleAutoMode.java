package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class LeftStartRightScaleAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public LeftStartRightScaleAutoMode(Alliance alliance) {
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
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset - AutoDistances.kBlueLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset
                            + AutoDistances.kBlueRightScaleY + Constants.kPlateWidth/2.0), 72.0));
            lastSegment.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset
                            + AutoDistances.kBlueRightScaleY + Constants.kPlateWidth/2.0), 0.0));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset - AutoDistances.kRedLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset
                            + AutoDistances.kRedRightScaleY + Constants.kPlateWidth/2.0), 72.0));
            lastSegment.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches,
                    -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset
                            + AutoDistances.kRedRightScaleY + Constants.kPlateWidth/2.0), 0.0));
        }

        ArrayList<Routine> routines = new ArrayList<Routine>();

        //Reset sensors before
        routines.add(new DriveSensorResetRoutine());

        //Drive path while moving elevator up and moving intake down
        ArrayList<Routine> inTransitRoutines = new ArrayList<>();
        inTransitRoutines.add(new DrivePathRoutine(new Path(path), false));
        inTransitRoutines.add(new IntakeDownRoutine());
        routines.add(new ParallelRoutine(inTransitRoutines));

        ArrayList<Routine> lastSegmentElevator = new ArrayList<>();
        lastSegmentElevator.add(new DrivePathRoutine(new Path(lastSegment), false));
        lastSegmentElevator.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 15));
        routines.add(new ParallelRoutine(lastSegmentElevator));

        //Open when everything is done to score
        routines.add(new IntakeOpenRoutine());

        return new SequentialRoutine(routines);
    }

	@Override
	public String getKey() {
		return mAlliance + " LEFT SCALE RIGHT";
	}
}
