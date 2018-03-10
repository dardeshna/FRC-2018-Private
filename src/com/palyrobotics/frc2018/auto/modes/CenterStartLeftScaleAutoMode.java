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
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class CenterStartLeftScaleAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public CenterStartLeftScaleAutoMode(Alliance alliance) {
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
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        if (mAlliance == Alliance.BLUE) {
            path.add(new Waypoint(new Translation2d(AutoDistances.kBlueLeftSwitchX - Constants.kRobotLengthInches,
                    AutoDistances.kBlueLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueLeftSwitchY/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kBlueLeftSwitchX + Constants.kRobotLengthInches,
                    AutoDistances.kBlueLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueLeftSwitchY/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kBlueLeftScaleX - 2.0 * Constants.kRobotLengthInches,
                    AutoDistances.kBlueLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueLeftScaleY
                            - AutoDistances.kScalePlateWidth/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kBlueLeftScaleX - Constants.kRobotLengthInches,
                    AutoDistances.kBlueLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kBlueLeftScaleY
                            - AutoDistances.kScalePlateWidth/2.0), 0.0));
        } else {
            path.add(new Waypoint(new Translation2d(AutoDistances.kRedLeftSwitchX - Constants.kRobotLengthInches,
                    AutoDistances.kRedLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kRedLeftSwitchY/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kRedLeftSwitchX + Constants.kRobotLengthInches,
                    AutoDistances.kRedLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kRedLeftSwitchY/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kRedLeftScaleX - 2.0 * Constants.kRobotLengthInches,
                    AutoDistances.kRedLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kRedLeftScaleY
                            - AutoDistances.kScalePlateWidth/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kRedLeftScaleX - Constants.kRobotLengthInches,
                    AutoDistances.kRedLeftToCenterY + Constants.kRobotWidthInches/2.0 - AutoDistances.kRedLeftScaleY
                            - AutoDistances.kScalePlateWidth/2.0), 0.0));
        }

        ArrayList<Routine> routines = new ArrayList<>();

        routines.add(new DriveSensorResetRoutine(1.0));

        //Drive path while moving elevator up and moving intake down
        ArrayList<Routine> inTransitRoutines = new ArrayList<>();
        inTransitRoutines.add(new DrivePathRoutine(new Path(path), false));
        inTransitRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 15));
        inTransitRoutines.add(new IntakeDownRoutine());
        routines.add(new ParallelRoutine(inTransitRoutines));

        //Open when everything is done to score
        routines.add(new IntakeOpenRoutine());

        return new SequentialRoutine(routines);
    }

	@Override
	public String getKey() {
		return mAlliance + " CENTER SCALE LEFT";
	}
}
