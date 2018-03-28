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

public class CenterStartMultiDifferent extends AutoModeBase {

    private Alliance mAlliance;

    public CenterStartMultiDifferent(AutoModeBase.Alliance alliance) {
        this.mAlliance = alliance;
    }

    //Point in between getting second cube and switch, used as a vertex to curve off of
    //-60, -20
    private Waypoint middleTransitPoint = new Waypoint(new Translation2d(-80.0, -20.0), 0.0);

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


        return new SequentialRoutine(routines);
    }

    public Routine getBackUpFromSwitch() {

        ArrayList<Routine> backUp = new ArrayList<>();

        //zero drive sensors
//        backUp.add(new DriveSensorResetRoutine(1.5));

        List<Waypoint> path = new ArrayList<>();

        path.add(new Waypoint(new Translation2d(1.515 * Constants.kRobotLengthInches - Constants.kCenterOfRotationOffsetFromFrontInches,
                -(AutoDistances.kFieldWidth - AutoDistances.kBlueLeftToCenterY - Constants.kRobotWidthInches/2.0) + AutoDistances.kBlueRightSwitchY
                        + AutoDistances.kSwitchPlateWidth/2.0), 80));
        path.add(new Waypoint(new Translation2d(40,0),0));
        path.add(new Waypoint(new Translation2d(35,0),0));


        backUp.add(new DrivePathRoutine(new Path(path), true));

        return new SequentialRoutine(backUp);
    }

    public Routine getDriveToAndIntakeCube(Alliance alliance) {

        ArrayList<Waypoint> path = new ArrayList<>();



        return new DriveUntilHasCubeRoutine(new DrivePathRoutine(path, false, 50.0, 27.5, 4.0));
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


    @Override
    public String getKey() {
        return mAlliance + " MULTI CENTER SWITCH RIGHT";
    }

}
