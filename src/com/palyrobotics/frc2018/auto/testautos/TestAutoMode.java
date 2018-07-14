package com.palyrobotics.frc2018.auto.testautos;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.*;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeCloseRoutine;
import com.palyrobotics.frc2018.behavior.routines.intake.IntakeDownRoutine;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.logger.Logger;

import java.util.ArrayList;
import java.util.logging.Level;

/**
 * Created by Nihar on 1/11/17. An AutoMode for running test autonomous
 */
public class TestAutoMode extends AutoModeBase {

	@Override
	public Routine getRoutine() {

		return getDrive();
//		return testF();
//		return getDrive();
	}

	@Override
	public String getKey() {
		return "Test Auto Mode";
	}

	@Override
	public String toString() {
		return "Test";
	}

	@Override
	public void prestart() {
		Logger.getInstance().logRobotThread(Level.FINE, "Starting TestAutoMode");
	}

	private Routine testF() {
		double power = 0.1;
		DriveSignal signal = DriveSignal.getNeutralSignal();
		signal.leftMotor.setVelocity(20 * Constants.kDriveSpeedUnitConversion, Gains.forsetiVelocity);
		signal.rightMotor.setVelocity(20 * Constants.kDriveSpeedUnitConversion, Gains.forsetiVelocity);

//		signal.leftMotor.setPercentOutput(0.4);
//		signal.rightMotor.setPercentOutput(0.4);
		return new TalonSRXRoutine(signal, false);
	}

	private SequentialRoutine getDrive() {
		Gains mShortGains = Gains.forsetiLongDriveMotionMagicGains;
		DriveSignal driveBackup = DriveSignal.getNeutralSignal();
		double driveBackupSetpoint = 60 * Constants.kDriveTicksPerInch;
		driveBackup.leftMotor.setMotionMagic(-1 * driveBackupSetpoint, mShortGains, (int) Gains.kForsetiLongDriveMotionMagicCruiseVelocity,
				(int) Gains.kForsetiLongDriveMotionMagicMaxAcceleration);
		driveBackup.rightMotor.setMotionMagic(driveBackupSetpoint, mShortGains, (int) Gains.kForsetiLongDriveMotionMagicCruiseVelocity,
				(int) Gains.kForsetiLongDriveMotionMagicMaxAcceleration);

		ArrayList<Routine> sequence = new ArrayList<>();
//		sequence.add(new IntakeDownRoutine());
		sequence.add(new DriveSensorResetRoutine(.1));
		sequence.add(new CascadingGyroEncoderTurnAngleRoutine(180));
//		sequence.add(new TalonSRXRoutine(driveBackup, false));

		return new SequentialRoutine(sequence);
	}
}
