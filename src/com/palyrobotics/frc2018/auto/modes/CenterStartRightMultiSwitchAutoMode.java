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
        if (mAlliance == Alliance.BLUE) {
        	startPoint = new Translation2d(AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches,
					-(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches/2.0)
					+ AutoDistances.kBlueRightSwitchY + AutoDistances.kSwitchPlateWidth/2.0);
        	midPoint =  new Translation2d(-85, AutoDistances.kBluePyramidFromRightY + AutoDistances.kBluePyramidWidth / 2.0 - AutoDistances.kBlueRightSwitchY - AutoDistances.kSwitchPlateWidth / 2.0);
        } else {
            startPoint = new Translation2d(AutoDistances.kRedRightSwitchX - Constants.kRobotLengthInches,
                    -(AutoDistances.kFieldWidth - AutoDistances.kRedLeftToCenterY - Constants.kRobotWidthInches/2.0)
                            + AutoDistances.kRedRightSwitchY + AutoDistances.kSwitchPlateWidth/2.0);
        	midPoint =  new Translation2d(-85, AutoDistances.kRedPyramidFromRightY + AutoDistances.kRedPyramidWidth / 2.0 - AutoDistances.kRedRightSwitchY - AutoDistances.kSwitchPlateWidth / 2.0);
        }
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

        ArrayList<Routine> prepareForSecondCube = new ArrayList<>();

        prepareForSecondCube.add(getPrepareForIntaking());
        prepareForSecondCube.add(getBackUpFromSwitch());

        //Back up and move elevator down
        routines.add(new ParallelRoutine(prepareForSecondCube));

        //Drive to and intake cube
        routines.add(getDriveToAndIntakeCube(mAlliance));

        routines.add(getReturnToSwitchPt1());

        routines.add(getReturnToSwitchPt2());

        routines.add(new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 2.0));

        return new SequentialRoutine(routines);
    }

    public Routine getBackUpFromSwitch() {
        ArrayList<Waypoint> path = new ArrayList<>();

        path.add(new Waypoint(new Translation2d(0.0, 0.0), 72.0, true));
        path.add(new Waypoint(new Translation2d(-40.0, 0.0), 72.0, true));
        path.add(new Waypoint(startPoint.translateBy(midPoint), 0.0));
        
        return new DrivePathRoutine(path, true, true);
    }

    public Routine getDriveToAndIntakeCube(Alliance alliance) {

        ArrayList<Waypoint> path = new ArrayList<>();
        
        path.add(new Waypoint(new Translation2d(0, 0), 35.0, true));

        if(alliance == Alliance.BLUE) {
            path.add(new Waypoint(startPoint.translateBy(new Translation2d(-AutoDistances.kBluePyramidLength,
                    AutoDistances.kBluePyramidFromRightY - AutoDistances.kBlueRightSwitchY - AutoDistances.kSwitchPlateWidth/2.0 + AutoDistances.kBluePyramidWidth/2.0)), 0.0));
        } else if(alliance == Alliance.RED) {
            path.add(new Waypoint(startPoint.translateBy(new Translation2d(-AutoDistances.kRedPyramidLength,
                    AutoDistances.kRedPyramidFromRightY - AutoDistances.kRedRightSwitchY - AutoDistances.kSwitchPlateWidth/2.0 + AutoDistances.kRedPyramidWidth/2.0)), 0.0));
        }

        return new DriveUntilHasCubeRoutine(new DrivePathRoutine(path, false, 50.0, 25.0, 4.0));
    }

    /**
     * Bring elevator and intake down
     *
     * @return
     */
    public Routine getPrepareForIntaking() {
        //Use this in parallel with backing up
        ArrayList<Routine> prepareForIntakingArrayList = new ArrayList<>();
        prepareForIntakingArrayList.add(new TimeoutRoutine(1));
        prepareForIntakingArrayList.add(new IntakeDownRoutine());
        prepareForIntakingArrayList.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches, 1.5));
        Routine prepareForIntakingRoutine = new SequentialRoutine(prepareForIntakingArrayList);
        return prepareForIntakingRoutine;
    }

    /**
     * Back up to get in position to drive in
     *
     * @return
     */
    public Routine getReturnToSwitchPt1() {
        ArrayList<Routine> returnToSwitchPt1ArrayList = new ArrayList<>();
        returnToSwitchPt1ArrayList.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorCubeInTransitPositionInches, 1.0));

        ArrayList<Waypoint> path = new ArrayList<>();
        
        path.add(new Waypoint(new Translation2d(0, 0), 50.0, true));
        path.add(new Waypoint(startPoint.translateBy(midPoint), 0.0));
        
        returnToSwitchPt1ArrayList.add(new DrivePathRoutine(path, true, 72.0, Constants.kPathFollowingLookahead, 4.0, true));

        return new ParallelRoutine(returnToSwitchPt1ArrayList);
    }

    /**
     * Drive into switch
     *
     * @return
     */
    public Routine getReturnToSwitchPt2() {
        ArrayList<Routine> returnToSwitchPt2ArrayList = new ArrayList<>();

        returnToSwitchPt2ArrayList.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 1.5));

        ArrayList<Waypoint> path = new ArrayList<>();
        
        path.add(new Waypoint(new Translation2d(0, 0), 60.0, true));
        path.add(new Waypoint(startPoint.translateBy(new Translation2d(-40.0, 0.0)), 60.0));
        path.add(new Waypoint(startPoint, 0.0));
        
        returnToSwitchPt2ArrayList.add(new DrivePathRoutine(path,  false, 72.0, 30.0, 4.0, true));

        return new ParallelRoutine(returnToSwitchPt2ArrayList);
    }

    @Override
    public String getKey() {
        return mAlliance + " CENTER SWITCH RIGHT SWITCH RIGHT";
    }

}
