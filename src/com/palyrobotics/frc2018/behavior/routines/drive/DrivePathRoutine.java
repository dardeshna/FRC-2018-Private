package com.palyrobotics.frc2018.behavior.routines.drive;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.robot.Robot;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.trajectory.Path;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

/**
 * Created by Nihar on 4/5/17.
 */
public class DrivePathRoutine extends Routine {
	private Path mPath;
	private ArrayList<Path.Waypoint> pathList;
	private double mLookAhead;
	private boolean mAddCurrentPosition;
	private double mStartSpeed;
	private boolean mInverted;
	private double mTolerance;

	/**
	 *
	 * @param path
	 *            Path to follow
	 * @param inverted whether or not to drive backwards
	 */
	public DrivePathRoutine(Path path, boolean inverted) {
		this.mPath = path;
		this.mLookAhead = Constants.kPathFollowingLookahead;
		this.mAddCurrentPosition = false;
		this.mStartSpeed = 72.0;
		this.mInverted = inverted;
		this.mTolerance = Constants.kPathFollowingTolerance;
	}

	/**
	 *
	 * @param path the path to follow
	 * @param lookAhead the lookahead distance desired
	 * @param inverted whether or not to drive backwards
	 */
	public DrivePathRoutine(Path path, boolean inverted, double lookAhead) {
		this.mPath = path;
		this.mLookAhead = lookAhead;
		this.mAddCurrentPosition = false;
		this.mStartSpeed = 0;
		this.mInverted = inverted;
		this.mTolerance = Constants.kPathFollowingTolerance;

	}

	public DrivePathRoutine(ArrayList<Path.Waypoint> pathList, boolean inverted, double startSpeed, boolean addCurrentPosition) {
		this.mPath = new Path(pathList);
	    this.pathList = pathList;
		this.mInverted = inverted;
		this.mStartSpeed = startSpeed;
		this.mAddCurrentPosition = addCurrentPosition;
		this.mTolerance = Constants.kPathFollowingTolerance;
	}

	public DrivePathRoutine(ArrayList<Path.Waypoint> pathList, boolean inverted, double startSpeed, boolean addCurrentPosition, double lookahead) {
		this.mPath = new Path(pathList);
		this.pathList = pathList;
		this.mInverted = inverted;
		this.mStartSpeed = startSpeed;
		this.mLookAhead = lookahead;
		this.mAddCurrentPosition = addCurrentPosition;
		this.mTolerance = Constants.kPathFollowingTolerance;
	}

	public DrivePathRoutine(ArrayList<Path.Waypoint> pathList, boolean inverted, double startSpeed, boolean addCurrentPosition, double lookahead, double tolerance) {
		this.mPath = new Path(pathList);
		this.pathList = pathList;
		this.mInverted = inverted;
		this.mStartSpeed = startSpeed;
		this.mLookAhead = lookahead;
		this.mAddCurrentPosition = addCurrentPosition;
		this.mTolerance = tolerance;
	}

	@Override
	public void start() {
		if(mAddCurrentPosition) {
			pathList.add(0, new Path.Waypoint(robotState.getLatestFieldToVehicle().getValue().getTranslation(), mStartSpeed));

			for(Path.Waypoint point : pathList) {
				System.out.println("Pos: " + point.position);
				System.out.println("Speed: " + point.speed);
			}

			mPath = new Path(pathList);
		}

		Logger.getInstance().logSubsystemThread(Level.INFO, "Starting Drive Path Routine");

		drive.setTrajectoryController(mPath, mLookAhead, mInverted, mTolerance);
	}

	@Override
	public Commands update(Commands commands) {
		commands.wantedDriveState = Drive.DriveState.ON_BOARD_CONTROLLER;
		return commands;
	}

	@Override
	public Commands cancel(Commands commands) {
		Logger.getInstance().logSubsystemThread(Level.INFO, "Drive Path Routine finished");
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
		List<Path.Waypoint> path = mPath.getWaypoints();
		enumeratedPath += "0,0,0\n";
		for (int i = 0; i < path.size(); i++) {
			enumeratedPath += (path.get(i).position.getX() +offsetX)  + "," + (path.get(i).position.getY() + offsetY) + "," + path.get(i).speed + "\n";
		}
		return enumeratedPath;
	}
}
