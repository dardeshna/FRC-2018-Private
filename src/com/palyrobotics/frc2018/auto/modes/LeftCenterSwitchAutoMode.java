package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.auto.AutoModeSelector;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class LeftCenterSwitchAutoMode extends AutoModeBase {
	
	public LeftCenterSwitchAutoMode() {
	}
	
	@Override
	public String toString() {
		return AutoModeSelector.mColor + " Right Switch Auto Mode";
	}

	@Override
	public void prestart() {
		
	}

	@Override
	public Routine getRoutine() {
		List<Waypoint> path = new ArrayList<>();
		path.add(new Waypoint(new Translation2d(0,0), 72.0));
		if(AutoModeSelector.mColor == AutoModeSelector.Color.BLUE) {
				path.add(new Waypoint(new Translation2d((AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches)/2.0,
						0.0), 72.0));
				path.add(new Waypoint(new Translation2d((AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches)/2.0,
						AutoDistances.kFieldWidth - AutoDistances.kCenterStartFromRight - AutoDistances.kBlueLeftSwitchY), 72.0));
				path.add(new Waypoint(new Translation2d((AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches),
						AutoDistances.kFieldWidth - AutoDistances.kCenterStartFromRight - AutoDistances.kBlueLeftSwitchY), 0.0));
		}
		else {
			path.add(new Waypoint(new Translation2d((AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches) / 2.0,
					0.0), 72.0));
			path.add(new Waypoint(new Translation2d((AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches) / 2.0,
					AutoDistances.kCenterStartFromRight - AutoDistances.kBlueRightSwitchY), 72.0));
			path.add(new Waypoint(new Translation2d((AutoDistances.kBlueRightSwitchX - Constants.kRobotLengthInches),
					AutoDistances.kCenterStartFromRight - AutoDistances.kBlueRightSwitchY), 0.0));
		}

		ArrayList<Routine> routines = new ArrayList<>();

		routines.add(new DriveSensorResetRoutine());
		routines.add(new DrivePathRoutine(new Path(path), false));

		return new SequentialRoutine(routines);
	}

}
