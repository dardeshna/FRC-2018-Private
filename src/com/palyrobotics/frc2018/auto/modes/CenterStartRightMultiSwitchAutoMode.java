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

    private Alliance mAlliance;

    public CenterStartRightMultiSwitchAutoMode(AutoModeBase.Alliance alliance) {
        this.mAlliance = alliance;
    }

    //Point in between getting second cube and switch, used as a vertex to curve off of
    private Waypoint middleTransitPoint = new Waypoint(new Translation2d(-60.0, -30.0), 0.0);

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
        routines.add(new CenterStartRightSwitchAutoMode(this.mAlliance).getRoutine());

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
        ArrayList<Routine> backUp = new ArrayList<>();

        //zero drive sensors
        backUp.add(new DriveSensorResetRoutine());

        List<Waypoint> path = new ArrayList<>();

        path.add(new Waypoint(new Translation2d(0.0, 0.0), 72.0));
        path.add(new Waypoint(new Translation2d(-40.0, 0.0), 72.0));
        path.add(middleTransitPoint);
        backUp.add(new DrivePathRoutine(new Path(path), true));

        return new SequentialRoutine(backUp);
    }

    public Routine getDriveToAndIntakeCube(Alliance alliance) {

        ArrayList<Waypoint> path = new ArrayList<>();

        if(alliance == Alliance.BLUE) {
            path.add(new Waypoint(new Translation2d(-AutoDistances.kBluePyramidLength + Constants.kCenterOfRotationOffsetFromFrontInches + Constants.kSquareCubeLength/2.0,
                    AutoDistances.kBluePyramidFromRightY - AutoDistances.kBlueRightSwitchY - Constants.kPlateWidth/2.0 + AutoDistances.kBluePyramidWidth/2.0), 0.0));
        } else if(alliance == Alliance.RED) {
            path.add(new Waypoint(new Translation2d(-AutoDistances.kRedPyramidLength + Constants.kCenterOfRotationOffsetFromFrontInches + Constants.kSquareCubeLength/2.0,
                    AutoDistances.kRedPyramidFromRightY - AutoDistances.kRedRightSwitchY - Constants.kPlateWidth/2.0 + AutoDistances.kRedPyramidWidth/2.0), 0.0));
        }

        return new DriveUntilHasCubeRoutine(new DrivePathRoutine(path, false, 50.0,  true, 27.5, 2.0));
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
        path.add(middleTransitPoint);
        returnToSwitchPt1ArrayList.add(new DrivePathRoutine(path, true, 72.0,  true, Constants.kPathFollowingLookahead, 2.0));

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
        path.add(new Waypoint(new Translation2d(0.0, 0.0), 0.0));
        returnToSwitchPt2ArrayList.add(new DrivePathRoutine(path,  false, 72.0, true, 30.0, 2.0));

        return new ParallelRoutine(returnToSwitchPt2ArrayList);
    }

    @Override
    public String getKey() {
        return mAlliance + " MULTI CENTER SWITCH RIGHT";
    }

}
