package com.palyrobotics.frc2018.behavior.routines.drive;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.util.trajectory.Path;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Nihar on 4/5/17.
 */
public class DrivePathRoutine extends Routine {
	private Path mPath;
	private boolean mInverted;

	/**
	 *
	 * @param path
	 *            Path to follow
	 */
	public DrivePathRoutine(Path path, boolean inverted) {
		this.mPath = path;
		this.mInverted = inverted;
	}

	@Override
	public void start() {
		drive.setTrajectoryController(mPath, mInverted);
	}

	@Override
	public Commands update(Commands commands) {
		commands.wantedDriveState = Drive.DriveState.ON_BOARD_CONTROLLER;
		return commands;
	}

	@Override
	public Commands cancel(Commands commands) {
		drive.setNeutral();
		commands.wantedDriveState = Drive.DriveState.NEUTRAL;
		return commands;
	}

	@Override
	public boolean finished() {
		return drive.controllerOnTarget();
	}

	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[] { drive };
	}

	@Override
	public String getName() {
		return "DrivePathRoutine";
	}

	@Override
	public String toString() {
		final int offsetX = 0;
		final int offsetY = 0;
		String enumeratedPath = "";
		List<Path.Waypoint> path = mPath.getmWaypoints();
		enumeratedPath += "0,0,0\n";
		for (int i = 0; i < path.size(); i++) {
			enumeratedPath += (path.get(i).position.getX() +offsetX)  + "," + (path.get(i).position.getY() + offsetY) + "," + path.get(i).speed + "\n";
		}
		return enumeratedPath;
	}
}
