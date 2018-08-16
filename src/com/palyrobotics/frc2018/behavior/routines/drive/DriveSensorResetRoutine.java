package com.palyrobotics.frc2018.behavior.routines.drive;

import java.util.logging.Level;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.robot.HardwareAdapter;
import com.palyrobotics.frc2018.subsystems.DriveSimulation;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;

/**
 * Created by EricLiu on 4/13/17.
 */
public class DriveSensorResetRoutine extends Routine {

	private double mTimeout;
	private long mStartTime;

	/**
	 *
	 * @param timeout seconds
	 */
	public DriveSensorResetRoutine(double timeout) {
		mTimeout = timeout;
	}

	@Override
	public void start() {
		System.out.println("Starting reset routine");
		this.mStartTime = System.currentTimeMillis();
		HardwareAdapter.DrivetrainHardware.resetSensors();
		DriveSimulation.getInstance().resetSensors();
		robotState.reset(0, new RigidTransform2d());
		robotState.drivePose.heading = 0.0;
        robotState.drivePose.leftEnc = 0.0;
        robotState.drivePose.rightEnc = 0.0;
        robotState.drivePose.lastHeading = 0.0;
		robotState.drivePose.lastLeftEnc = 0.0;
		robotState.drivePose.lastRightEnc = 0.0;
	}

	@Override
	public Commands update(Commands commands) {
		Commands output = commands.copy();
		return output;
	}

	@Override
	public Commands cancel(Commands commands) {
		System.out.println("finished reset routine");
		Commands output = commands.copy();
		return output;
	}

	@Override
	public boolean finished() {
		if(System.currentTimeMillis() - mStartTime > mTimeout * 1000) {
			Logger.getInstance().logRobotThread(Level.WARNING, "Drive sensor reset routine timed out!");
//			System.out.println("Time: " + String.valueOf(System.currentTimeMillis() - mStartTime) + " timed out");
			return true;
		} else if(Math.abs(drive.getPose().leftEnc) <= Constants.kAcceptableEncoderZeroError
				&& Math.abs(drive.getPose().rightEnc) <= Constants.kAcceptableEncoderZeroError
				&& Math.abs(drive.getPose().heading) <= Constants.kAcceptableGyroZeroError) {
//			System.out.println("Time: " + String.valueOf(System.currentTimeMillis() - mStartTime));
//			System.out.println("Good error on drive sensors, finished");
			return true;
		}
//		System.out.println("just for shits and giggles");
		return false;
	}

	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[] { drive };
	}

	@Override
	public String getName() {
		return "DriveSensorResetRoutine";
	}
}
