package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeSensorStopRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class CenterStartLeftSwitchAutoMode extends AutoModeBase {

	private double dy;
	private double dx;
	public static Waypoint end;

	static {
		double dy = (mDistances.kFieldWidth/2 - mDistances.kLeftSwitchY) * .55;
//		dy *= -1;
		double dx = mDistances.kLeftSwitchX - Constants.kRobotLengthInches - Constants.kNullZoneAllowableBack;
		CenterStartLeftSwitchAutoMode.end = new Waypoint(new Translation2d(dx, dy), 0);
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
		path.add(new Waypoint(new Translation2d(0, 0), 190));

		path.add(new Waypoint(new Translation2d(dx/2, dy/2), 120));
		path.add(CenterStartLeftSwitchAutoMode.end);

		ArrayList<Routine> routines = new ArrayList<>();

		routines.add(new DriveSensorResetRoutine(50/1000 * 3));

		ArrayList<Routine> inTransitRoutines = new ArrayList<>();
		inTransitRoutines.add(new DrivePathRoutine(new Path(path), false));

		ArrayList<Routine> scoreRoutines = new ArrayList<>();
		scoreRoutines.add(new IntakeDownRoutine());
		scoreRoutines.add(new TimeoutRoutine(Constants.kSwitchAutoWaitBeforeElevatorRaiseTimeSeconds));
		scoreRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 1.5));
		inTransitRoutines.add(new SequentialRoutine(scoreRoutines));

		routines.add(new ParallelRoutine(inTransitRoutines));

		//Expel when everything is done to score
		routines.add(new IntakeSensorStopRoutine(Intake.WheelState.VAULT_EXPELLING, .2));
//		routines.add(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1.0));

		return new SequentialRoutine(routines);
	}

	@Override
	public String getKey() {
		return mAlliance + " CENTER SWITCH RIGHT";
	}
}
