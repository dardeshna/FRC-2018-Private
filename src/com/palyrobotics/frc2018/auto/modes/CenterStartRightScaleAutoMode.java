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
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class CenterStartRightScaleAutoMode extends AutoModeBase {

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
        path.add(new Path.Waypoint(new Translation2d(0, 0), 72.0));
        if (mAlliance == Alliance.BLUE) {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches)
                            + AutoDistances.kBlueRightSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches)
                            + AutoDistances.kBlueRightSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches/2.0)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches/2.0)
                            + AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightSwitchX - Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kRedLeftToCenterY - Constants.kRobotWidthInches)
                            + AutoDistances.kRedRightSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightSwitchX + Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kRedLeftToCenterY - Constants.kRobotWidthInches)
                            + AutoDistances.kRedRightSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - 2.0 * Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kRedLeftToCenterY - Constants.kRobotWidthInches/2.0)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kRedLeftToCenterY - Constants.kRobotWidthInches/2.0)
                            + AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth/2.0), 0.0));
        }

        ArrayList<Routine> routines = new ArrayList<>();

        routines.add(new DriveSensorResetRoutine(1.0));

		ArrayList<Routine> inTransitRoutines = new ArrayList<>();
		inTransitRoutines.add(new DrivePathRoutine(new Path(path), false));

		/*ArrayList<Routine> scoreRoutines = new ArrayList<>();
		scoreRoutines.add(new IntakeDownRoutine());
		scoreRoutines.add(new TimeoutRoutine(Constants.kScaleAutoWaitBeforeElevatorRaiseTimeSeconds));
	 	scoreRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 3.0));
		inTransitRoutines.add(new SequentialRoutine(scoreRoutines));*/

		routines.add(new ParallelRoutine(inTransitRoutines));

		//Expel when everything is done to score
		//routines.add(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 3.0));

		return new SequentialRoutine(routines);
    }

	@Override
	public String getKey() {
		return mAlliance + " CENTER SCALE RIGHT";
	}
}
