package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class LeftStartLeftScaleAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public LeftStartLeftScaleAutoMode(Alliance alliance) {
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
        path.add(new Path.Waypoint(new Translation2d(0.0, 0.0), 72.0));
        if(mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueLeftSwitchX - Constants.kRobotLengthInches,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset - AutoDistances.kBlueLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueLeftSwitchX + Constants.kRobotLengthInches,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset - AutoDistances.kBlueLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueLeftScaleX - 2.0*Constants.kRobotLengthInches,
                    (Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset)
                            - AutoDistances.kBlueLeftScaleY - Constants.kPlateWidth/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueLeftScaleX - Constants.kRobotLengthInches,
                    (Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset)
                            - AutoDistances.kBlueLeftScaleY - Constants.kPlateWidth/2.0), 0.0));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedLeftSwitchX - Constants.kRobotLengthInches,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset - AutoDistances.kRedLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedLeftSwitchX + Constants.kRobotLengthInches,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset - AutoDistances.kRedLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedLeftScaleX - 2.0*Constants.kRobotLengthInches,
                    (Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset)
                            - AutoDistances.kRedLeftScaleY - Constants.kPlateWidth/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedLeftScaleX - Constants.kRobotLengthInches,
                    (Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset)
                            - AutoDistances.kRedLeftScaleY - Constants.kPlateWidth/2.0), 0.0));
        }
        ArrayList<Routine> driveRoutines = new ArrayList<>();
        driveRoutines.add(new DriveSensorResetRoutine());
        driveRoutines.add(new DrivePathRoutine(new Path(path), false));
        driveRoutines.add(new IntakeOpenRoutine());
        Routine driveRoutine = new SequentialRoutine(driveRoutines);
        
        ArrayList<Routine> elevatorRoutines = new ArrayList<>();
        elevatorRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopPositionInches, 15));
        Routine elevatorRoutine = new SequentialRoutine(elevatorRoutines);
        
        ArrayList<Routine> routines = new ArrayList<Routine>();
        routines.add(driveRoutine);
        routines.add(elevatorRoutine);
        return new ParallelRoutine(routines);
    }


	@Override
	public String getKey() {
		return mAlliance + " LEFT SCALE LEFT";
	}
}
