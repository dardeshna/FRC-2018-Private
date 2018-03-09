package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.GyroMotionMagicTurnAngleRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class LeftStartRightSwitchAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public LeftStartRightSwitchAutoMode(Alliance alliance) {
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
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset - AutoDistances.kBlueLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueLeftSwitchX + AutoDistances.kSwitchPlateLength + Constants.kRobotLengthInches/2.0
                    + Constants.kSquareCubeLength, -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kBlueLeftCornerOffset
                            + AutoDistances.kBlueRightSwitchY + AutoDistances.kSwitchPlateWidth/2.0), 0.0));
        } else {
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedScaleSwitchMidlineX - Constants.kRobotLengthInches/2.0,
                    Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset - AutoDistances.kRedLeftSwitchY/2.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedLeftSwitchX + AutoDistances.kSwitchPlateLength + Constants.kRobotLengthInches/2.0
                    + Constants.kSquareCubeLength, -AutoDistances.kFieldWidth + Constants.kRobotWidthInches/2.0 + AutoDistances.kRedLeftCornerOffset
                            + AutoDistances.kRedRightSwitchY + AutoDistances.kSwitchPlateWidth/2.0), 0.0));
        }

        ArrayList<Routine> routines = new ArrayList<Routine>();

        //Reset sensors before
        routines.add(new DriveSensorResetRoutine());

        ArrayList<Routine> inTransitRoutines = new ArrayList<>();
        ArrayList<Routine> driveRoutines = new ArrayList<>();

        //Drive there then turn angle to face
        driveRoutines.add(new DrivePathRoutine(new Path(path), false));
        //TODO: which turn angle? also is the angle correct?
        driveRoutines.add(new GyroMotionMagicTurnAngleRoutine(-90));

        //Get there, raise elevator, intake down
        inTransitRoutines.add(new SequentialRoutine(driveRoutines));
        inTransitRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorTopBottomDifferenceInches, 15));
        inTransitRoutines.add(new IntakeDownRoutine());
        routines.add(new ParallelRoutine(inTransitRoutines));

        //Spit out the cube at some speed to get it in the switch past the cube on the ground
        routines.add(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, Constants.kExpelToScoreTime));


        return new SequentialRoutine(routines);
    }

	@Override
	public String getKey() {
		return mAlliance + " LEFT SWITCH RIGHT";
	}
}
