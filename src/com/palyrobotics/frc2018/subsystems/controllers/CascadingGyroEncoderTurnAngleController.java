package com.palyrobotics.frc2018.subsystems.controllers;

import java.util.logging.Level;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.subsystems.Drive.DriveController;
import com.palyrobotics.frc2018.util.DriveSignal;
import com.palyrobotics.frc2018.util.Pose;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import com.palyrobotics.frc2018.util.logger.Logger;

public class CascadingGyroEncoderTurnAngleController implements DriveController {

    private double mTargetHeading;
    private Pose mCachedPose;
    
    private double mLeftTarget;
    private double mRightTarget;
    
    private TalonSRXOutput mLeftOutput;
    private TalonSRXOutput mRightOutput;
    
    //Error measurements for angle-to-velocity PID
    private double mErrorIntegral;
    private double mErrorDerivative;
    private double mLastError;

    public CascadingGyroEncoderTurnAngleController(Pose priorSetpoint, double angle) {
        mTargetHeading = priorSetpoint.heading + angle;
        mCachedPose = priorSetpoint;
        
        mLeftOutput = new TalonSRXOutput();
        mRightOutput = new TalonSRXOutput();
        
        mErrorIntegral = 0;
        mLastError = angle;
    }

    @Override
    public DriveSignal update(RobotState state) {

        mCachedPose = state.drivePose;
        
        if (mCachedPose == null) {
        	Logger.getInstance().logSubsystemThread(Level.WARNING, "CascadingGyroEncoderTurnAngle", "Cached pose is null!");
        	return DriveSignal.getNeutralSignal();
        } else {
        	 double currentHeading = mCachedPose.heading;
             double error = mTargetHeading - currentHeading;
             mErrorIntegral += error;
             mErrorDerivative = (mLastError - error) * Constants.kNormalLoopsDt;

             //Manually calculate PID output for velocity loop
             mLeftTarget = -1 * (Gains.kForsetiCascadingTurnkP * error + Gains.kForsetiCascadingTurnkI * mErrorIntegral + Gains.kForsetiCascadingTurnkD * mErrorDerivative);
             mRightTarget = 1 * (Gains.kForsetiCascadingTurnkP * error + Gains.kForsetiCascadingTurnkI * mErrorIntegral + Gains.kForsetiCascadingTurnkD * mErrorDerivative);

             mLeftOutput.setVelocity(mLeftTarget, Gains.forsetiVelocity);
             mRightOutput.setVelocity(mRightTarget, Gains.forsetiVelocity);
             
             mLastError = error;
             
             return new DriveSignal(mLeftOutput, mRightOutput);
        }
    }

    @Override
    public Pose getSetpoint() {
        return null;
    }

    @Override
    public boolean onTarget() {
        if (mCachedPose == null) {
        	Logger.getInstance().logSubsystemThread(Level.WARNING, "CascadingGyroEncoderTurnAngle", "Cached pose is null!");
        	return false;
        } else {
        	return Math.abs(mLastError) < Constants.kAcceptableTurnAngleError &&
        			Math.abs(mCachedPose.leftEncVelocity) < Constants.kAcceptableDriveVelocityError &&
        			Math.abs(mCachedPose.rightEncVelocity) < Constants.kAcceptableDriveVelocityError;
        }
        
    }
}
