package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.auto.AutoModeBase.Alliance;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.behavior.routines.elevator.ElevatorCustomPositioningRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeOpenRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeWheelRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;
import com.palyrobotics.frc2018.util.trajectory.Path;

import java.util.ArrayList;
import java.util.List;

public class CenterStartLeftMultiSwitchAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public CenterStartLeftMultiSwitchAutoMode(AutoModeBase.Alliance alliance) {
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
            path.add(new Waypoint(new Translation2d(2.0 * Constants.kRobotLengthInches,
                    AutoDistances.kBlueLeftToCenterY + Constants.kRobotWidthInches
                            - AutoDistances.kBlueLeftSwitchY - Constants.kPlateWidth/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kBlueLeftSwitchX - Constants.kRobotLengthInches,
                    AutoDistances.kBlueLeftToCenterY + Constants.kRobotWidthInches
                            - AutoDistances.kBlueLeftSwitchY - Constants.kPlateWidth/2.0), 0.0));
        } else {
            path.add(new Waypoint(new Translation2d(2.0 * Constants.kRobotLengthInches,
                    AutoDistances.kRedLeftToCenterY + Constants.kRobotWidthInches
                            - AutoDistances.kRedLeftSwitchY - Constants.kPlateWidth/2.0), 72.0));
            path.add(new Waypoint(new Translation2d(AutoDistances.kRedLeftSwitchX - Constants.kRobotLengthInches,
                    AutoDistances.kRedLeftToCenterY + Constants.kRobotWidthInches
                            - AutoDistances.kRedLeftSwitchY - Constants.kPlateWidth/2.0), 0.0));
        }

        ArrayList<Routine> routines = new ArrayList<>();

        routines.add(new DriveSensorResetRoutine());

        //Drive path while moving elevator up and moving intake down
        ArrayList<Routine> inTransitRoutines = new ArrayList<>();
        inTransitRoutines.add(new DrivePathRoutine(new Path(path), false));
        inTransitRoutines.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 15));
        inTransitRoutines.add(new IntakeDownRoutine());
        routines.add(new ParallelRoutine(inTransitRoutines));

        //Open when everything is done to score
        routines.add(getExpelDrop());

        ArrayList<Routine> secondCube = new ArrayList<>();
        secondCube.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorBottomPositionInches, 15));
        // back up to the right
        secondCube.add(getBackupToSwitch());
        routines.add(new ParallelRoutine(secondCube));

        ArrayList<Routine> driveIntake = new ArrayList<>();
        driveIntake.add(new IntakeWheelRoutine(Intake.WheelState.INTAKING, 1));
        driveIntake.add(getDriveForward());
        routines.add(new ParallelRoutine(driveIntake));

        ArrayList<Routine> driveBack = new ArrayList<>();
        driveBack.add(new IntakeWheelRoutine(Intake.WheelState.IDLE, 1));
        driveBack.add(getBackupToSwitch());

        routines.add(new ParallelRoutine(driveBack));

        ArrayList<Routine> prepareForDrop = new ArrayList<>();
        prepareForDrop.add(new ElevatorCustomPositioningRoutine(Constants.kElevatorSwitchPositionInches, 15));
        prepareForDrop.add(goBackToSwitch());

        routines.add(new ParallelRoutine(prepareForDrop));
        routines.add(getExpelDrop());

        return new SequentialRoutine(routines);
    }

    public ParallelRoutine getExpelDrop() {
        return new ParallelRoutine(new ArrayList<Routine>() {{
            new IntakeOpenRoutine();
            new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1);
        }});
    }

    /**
     * Distance:
     * - [(field width)/2 - distance from side to switch + pyramid stack width /2]
     * - [pyramid stack width + some arbitrary constant to not hit the switch]
     *
     * @return
     */
    public DrivePathRoutine getBackupToSwitch() {
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        if (mAlliance == Alliance.BLUE) {
            path.add(new Waypoint(new Translation2d(-AutoDistances.pyramidStackX - Constants.kRobotLengthInches/2,
                    -AutoDistances.kFieldWidth/2 + AutoDistances.kBlueLeftScaleY - Constants.kRobotWidthInches), 0.0));
        } else {
            path.add(new Waypoint(new Translation2d(-AutoDistances.pyramidStackX - Constants.kRobotLengthInches/2,
                    -AutoDistances.kFieldWidth/2 + AutoDistances.kRedLeftScaleY - Constants.kRobotWidthInches), 0.0));
        }

        return new DrivePathRoutine(new Path(path), false);
    }

    /**
     * Distance:
     * - [(field width)/2 - distance from side to switch + pyramid stack width /2]
     * - [pyramid stack width + some arbitrary constant to not hit the switch]
     *
     * @return
     */
    public DrivePathRoutine goBackToSwitch() {
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        if (mAlliance == Alliance.BLUE) {
            path.add(new Waypoint(new Translation2d(AutoDistances.pyramidStackX + Constants.kRobotLengthInches/2,
                    AutoDistances.kFieldWidth/2 - AutoDistances.kBlueLeftScaleY + Constants.kRobotWidthInches), 0.0));
        } else {
            path.add(new Waypoint(new Translation2d(AutoDistances.pyramidStackX + Constants.kRobotLengthInches/2,
                    AutoDistances.kFieldWidth/2 - AutoDistances.kRedLeftScaleY + Constants.kRobotWidthInches), 0.0));
        }

        return new DrivePathRoutine(new Path(path), false);
    }

    public DrivePathRoutine getDriveForward() {
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        path.add(new Waypoint(new Translation2d(12,0), 0));
        return new DrivePathRoutine(new Path(path), false);
    }


    public DrivePathRoutine getDriveBack() {
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Translation2d(0, 0), 72.0));
        path.add(new Waypoint(new Translation2d(12,0), 0));
        return new DrivePathRoutine(new Path(path), false);
    }

    @Override
    public String getKey() {
        return mAlliance + " Center Switch Left";
    }

}
