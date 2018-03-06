package com.palyrobotics.frc2018.behavior.routines.drive;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.robot.HardwareAdapter;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Created by EricLiu on 4/13/17.
 */
public class DriveSensorResetRoutine extends Routine {

	public DriveSensorResetRoutine() {
	}

	@Override
	public void start() {
	}

	@Override
	public Commands update(Commands commands) {
		HardwareAdapter.getInstance().getDrivetrain().resetSensors();
		robotState.reset(0, new RigidTransform2d());
		Commands output = commands.copy();
		return output;
	}

	@Override
	public Commands cancel(Commands commands) {
		Commands output = commands.copy();
		return output;
	}

	@Override
	public boolean finished() {
		if(Math.abs(drive.getPose().leftEnc) <= Constants.kAcceptableEncoderZeroError
				&& Math.abs(drive.getPose().rightEnc) <= Constants.kAcceptableEncoderZeroError
				&& Math.abs(drive.getPose().heading) <= Constants.kAcceptableGyroZeroError) {
			return true;
		} else
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
