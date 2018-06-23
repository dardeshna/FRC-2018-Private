package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveUntilHasCubeRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeSensorStopRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;
import com.palyrobotics.frc2018.util.trajectory.Path;

import java.util.ArrayList;
import java.util.List;

public class CenterStartRightMultiSwitchAutoMode extends AutoModeBase {

	private Translation2d startPoint;
	private Translation2d midPoint;
	
    public CenterStartRightMultiSwitchAutoMode() {
    }

    //Point in between getting second cube and switch, used as a vertex to curve off of
    //-60, -20

    @Override
    public String toString() {
        return mAlliance + this.getClass().toString();
    }

    @Override
    public void prestart() {

    }

    @Override
    public Routine getRoutine() {
        ArrayList<Routine> routines = new ArrayList<>();

        //Initial cube score
        routines.add(new CenterStartRightSwitchAutoMode().getRoutine());

        ArrayList<Routine> secondCubeRoutine = new ArrayList<>();

        secondCubeRoutine.add(getFirstBackup());
        secondCubeRoutine.add(driveForward());

        routines.add(new SequentialRoutine(secondCubeRoutine));
        routines.add(driveBackToSwitchAndExpel());


        return new SequentialRoutine(routines);
    }

    public Waypoint getFirstBackUpPoint() {
        double backX = mDistances.kRightSwitchX - Constants.kRobotLengthInches - mDistances.kPyramidLength * 1.68;
        return new Waypoint(new Translation2d(backX,0), 0);
    }

    public Routine getFirstBackup() {
        List<Waypoint> path = new ArrayList<>();
        Waypoint cp = new Waypoint(CenterStartRightSwitchAutoMode.end.position, 100);
        path.add(cp);

        path.add(getFirstBackUpPoint());

        ArrayList<Routine> dropElevator = new ArrayList<>();
        dropElevator.add(new TimeoutRoutine(.2));
        dropElevator.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches, 1.1));

        return new ParallelRoutine(new DrivePathRoutine(new Path(path), true),new SequentialRoutine(dropElevator));
    }

    public Routine driveForward() {
        List<Waypoint> path = new ArrayList<>();
        Waypoint cp = new Waypoint(getFirstBackUpPoint().position, 70);
        path.add(cp);

        double backX = mDistances.kRightSwitchX - Constants.kRobotLengthInches - mDistances.kPyramidLength*1.1;
        path.add(new Waypoint(new Translation2d(backX,0), 0));

        return new DriveUntilHasCubeRoutine(new DrivePathRoutine(new Path(path), false));
    }

    public Routine driveBackToSwitchAndExpel() {
        ArrayList<Routine> routines = new ArrayList<>();

        List<Waypoint> path = new ArrayList<>();
        double backX = mDistances.kRightSwitchX - Constants.kRobotLengthInches - mDistances.kPyramidLength*1.1;
        path.add(new Waypoint(new Translation2d(backX,0), 80));

        path.add(getFirstBackUpPoint());
        routines.add(new DrivePathRoutine(new Path(path), true));


        // The last part of driving to the switch is done in two parts.
        // Let the total translation required be a dX and a dY.  To ensure
        // that we end with a correct angle, set a point from the current point translated by dx/3 and dy*2/3.
        // the last point is just a translation of dy and dx.

        List<Waypoint> secondPath = new ArrayList<>();
        secondPath.add(new Waypoint(getFirstBackUpPoint().position.translateBy(new Translation2d(Constants.kSquareCubeLength,0)), 100));

        // NOTE: THE CONSTANT AT THE END NEEDS TO BE HIGHER BECAUSE THE POSITION ESTIMATOR IS _BAD_
        double dy = (mDistances.kFieldWidth/2 - mDistances.kRightSwitchY)/2 * .96;

        dy *= -1;

        double dx = (mDistances.kRightSwitchX - Constants.kRobotLengthInches - Constants.kNullZoneAllowableBack) -
                (mDistances.kRightSwitchX - Constants.kRobotLengthInches - mDistances.kPyramidLength * 1.68);

        secondPath.add(new Waypoint(getFirstBackUpPoint().position.translateBy(new Translation2d(dx/3, dy*2/3)), 80));
        secondPath.add(new Waypoint(getFirstBackUpPoint().position.translateBy(new Translation2d(dx, dy)), 0));

        routines.add(new ParallelRoutine(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 1.2),
                    new DrivePathRoutine(new Path(secondPath), false)));
        routines.add(new IntakeWheelRoutine(Intake.WheelState.VAULT_EXPELLING, .3));

        return new SequentialRoutine(routines);
    }

    @Override
    public String getKey() {
        return mAlliance + " CENTER SWITCH RIGHT SWITCH RIGHT";
    }

}
