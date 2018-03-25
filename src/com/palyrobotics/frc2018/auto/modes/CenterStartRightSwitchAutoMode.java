package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.*;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class CenterStartRightSwitchAutoMode extends AutoModeBase {

	public CenterStartRightSwitchAutoMode(Alliance alliance) {
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
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        if (mAlliance == Alliance.BLUE) {
        	path.add(new Waypoint(new Translation2d(1.2 * Constants.kRobotLengthInches - Constants.kCenterOfRotationOffsetFromFrontInches, 0), 72.0));
            path.add(new Waypoint(new Translation2d(1.515 * Constants.kRobotLengthInches - Constants.kCenterOfRotationOffsetFromFrontInches,
					-(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches/2.0) + AutoDistances.kBlueRightSwitchY
					+ AutoDistances.kSwitchPlateWidth/2.0), 60.0));
			path.add(new Waypoint(new Translation2d(AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches,
					-(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches/2.0)
							+ AutoDistances.kBlueRightSwitchY + AutoDistances.kSwitchPlateWidth/2.0), 0.0));
        } else {
			path.add(new Waypoint(new Translation2d(1.2 * Constants.kRobotLengthInches - Constants.kCenterOfRotationOffsetFromFrontInches, 0), 72.0));
			path.add(new Waypoint(new Translation2d(1.515 * Constants.kRobotLengthInches - Constants.kCenterOfRotationOffsetFromFrontInches,
					-(AutoDistances.kFieldWidth - AutoDistances.kRedLeftToCenterY - Constants.kRobotWidthInches/2.0) + AutoDistances.kRedRightSwitchY
							+ AutoDistances.kSwitchPlateWidth/2.0), 60.0));
			path.add(new Waypoint(new Translation2d(AutoDistances.kRedRightSwitchX - Constants.kRobotLengthInches,
					-(AutoDistances.kFieldWidth - AutoDistances.kRedLeftToCenterY - Constants.kRobotWidthInches/2.0)
							+ AutoDistances.kRedRightSwitchY + AutoDistances.kSwitchPlateWidth/2.0), 0.0));
        }

        ArrayList<Routine> routines = new ArrayList<>();

        routines.add(new DriveSensorResetRoutine(1.0));

		ArrayList<Routine> inTransitRoutines = new ArrayList<>();
		inTransitRoutines.add(new DrivePathRoutine(new Path(path), false));

		ArrayList<Routine> scoreRoutines = new ArrayList<>();
		scoreRoutines.add(new IntakeDownRoutine());
		scoreRoutines.add(new TimeoutRoutine(Constants.kSwitchAutoWaitBeforeElevatorRaiseTimeSeconds));
	 	scoreRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 1.5));
		inTransitRoutines.add(new SequentialRoutine(scoreRoutines));

		routines.add(new ParallelRoutine(inTransitRoutines));

		//Expel when everything is done to score
		routines.add(new IntakeSensorStopRoutine(Intake.WheelState.EXPELLING, 1.5));
//		routines.add(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1.0));

		routines.add(new DriveSensorResetRoutine(1.0));
		return new SequentialRoutine(routines);
	}

	@Override
	public String getKey() {
		return mAlliance + " CENTER SWITCH RIGHT";
	}
}
