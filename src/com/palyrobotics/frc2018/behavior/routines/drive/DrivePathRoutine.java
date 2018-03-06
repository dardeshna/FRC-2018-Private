package com.palyrobotics.frc2018.behavior.routines.drive;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.robot.Robot;
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
	private ArrayList<Path.Waypoint> pathList;
	private double mLookAhead;
	private boolean mAddCurrentPosition;
	private double mStartSpeed;
	private boolean mInverted;

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
	}

	/**
	 *
	 * @param path the path to follow
	 * @param lookAhead the lookahead distance desired
	 * @param inverted whether or not to drive backwards
	 */
	public DrivePathRoutine(Path path, double lookAhead, boolean inverted) {
		this.mPath = path;
		this.mLookAhead = lookAhead;
		this.mAddCurrentPosition = false;
		this.mStartSpeed = 0;
		this.mInverted = inverted;
	}

	/**
	 *
	 * @param path the path to follow
	 * @param lookAhead the lookahead distance desired
	 * @param addCurrentPosition whether or not to add your current point to the start of the path
	 * @param startSpeed your starting speed
	 * @param inverted whether or not to drive backwards
	 */
	public DrivePathRoutine(Path path, double lookAhead, boolean addCurrentPosition, double startSpeed, boolean inverted) {
		this.mPath = path;
		this.mLookAhead = lookAhead;
		this.mAddCurrentPosition = addCurrentPosition;
		this.mStartSpeed = startSpeed;
		this.mInverted = inverted;
	}

	/**
	 *
	 * @param path the path to follow
	 * @param addCurrentPosition whether or not to add your current point to the start of the path
	 * @param currentSpeed your start speed
	 * @param inverted whether or not to drive backwards
	 */
	public DrivePathRoutine(Path path, boolean addCurrentPosition, double currentSpeed, boolean inverted) {
		this.mPath = path;
		this.mAddCurrentPosition = addCurrentPosition;
		this.mStartSpeed = currentSpeed;
		this.mInverted = inverted;
	}

	public DrivePathRoutine(ArrayList<Path.Waypoint> pathList, boolean inverted, boolean addCurrentPosition) {
		this.pathList = pathList;
		this.mInverted = inverted;
		this.mAddCurrentPosition = addCurrentPosition;
	}

	@Override
	public void start() {
		if(mAddCurrentPosition) {
			pathList.add(0, new Path.Waypoint(robotState.getLatestFieldToVehicle().getValue().getTranslation(), mStartSpeed));
			mPath = new Path(pathList);
		}

		System.out.println("AFTER");
		for (Path.Waypoint point : mPath.getWaypoints()) {
			System.out.println("pos: " + point.position.toString());
			System.out.println("speed: " + point.speed);
		}

		drive.setTrajectoryController(mPath, mLookAhead, mInverted);
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
		System.out.println(drive.controllerOnTarget());
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
