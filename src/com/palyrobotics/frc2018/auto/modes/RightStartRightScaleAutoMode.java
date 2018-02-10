package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RightStartRightScaleAutoMode extends AutoModeBase {

    private Alliance mAlliance;

    public RightStartRightScaleAutoMode(Alliance alliance) {
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
            path.add(new Path.Waypoint(new Translation2d((AutoDistances.kBlueRightScaleX
                    - Constants.kRobotLengthInches) / 2, 0.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d((AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches) / 2,
                    AutoDistances.kBlueRightScaleY + (Constants.kPlateWidth / 2) - AutoDistances.kBlueRightCornerOffset
                            - (Constants.kPlateWidth / 2)), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kBlueRightScaleX - Constants.kRobotLengthInches,
                    AutoDistances.kBlueRightScaleY + (Constants.kPlateWidth / 2) - AutoDistances.kBlueRightCornerOffset
                            - (Constants.kPlateWidth / 2)), 0.0));
        } else {
            path.add(new Path.Waypoint(new Translation2d((AutoDistances.kRedRightScaleX
                    - Constants.kRobotLengthInches) / 2, 0.0), 72.0));
            path.add(new Path.Waypoint(new Translation2d((AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches) / 2,
                    AutoDistances.kRedRightScaleY + (Constants.kPlateWidth / 2) - AutoDistances.kRedRightCornerOffset
                            - (Constants.kPlateWidth / 2)), 72.0));
            path.add(new Path.Waypoint(new Translation2d(AutoDistances.kRedRightScaleX - Constants.kRobotLengthInches,
                    AutoDistances.kRedRightScaleY + (Constants.kPlateWidth / 2) - AutoDistances.kRedRightCornerOffset
                            - (Constants.kPlateWidth / 2)), 0.0));
        }
        ArrayList<Routine> routines = new ArrayList<>();
        routines.add(new DriveSensorResetRoutine());
        routines.add(new DrivePathRoutine(new Path(path), false));

        return new SequentialRoutine(routines);
    }

	@Override
	public String getKey() {
		return mAlliance + " RIGHT SCALE RIGHT";
	}
}
