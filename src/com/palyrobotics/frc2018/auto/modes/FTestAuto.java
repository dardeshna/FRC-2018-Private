package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.EncoderTurnAngleRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.TalonSRXRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.TimedDriveRoutine;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.logger.Logger;

import java.util.ArrayList;
import java.util.logging.Level;

/**
 * Created by Nihar on 1/11/17. An AutoMode for running test autonomous
 */
public class FTestAuto extends AutoModeBase {

    @Override
    public Routine getRoutine() {
        return new TimedDriveRoutine(0.2, 6.5);
//        return getDrive();
    }

    @Override
    public String toString() {
        return "Test";
    }

    @Override
    public void prestart() {
        Logger.getInstance().logRobotThread(Level.FINE, "Starting TestAutoMode");
    }

    private ParallelRoutine getDrive() {
        Gains mShortGains = Gains.forsetiShortDriveMotionMagicGains;

        DriveSignal driveBackup = DriveSignal.getNeutralSignal();
        driveBackup.leftMotor.setPercentOutput(.48);
        driveBackup.rightMotor.setPercentOutput(.48);
        ArrayList<Routine> sequence = new ArrayList<>();

        sequence.add(new TalonSRXRoutine(driveBackup, true));
        sequence.add(new TimeoutRoutine(10));


        return new ParallelRoutine(sequence);
    }
}
