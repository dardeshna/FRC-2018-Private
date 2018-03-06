package com.palyrobotics.frc2018.util;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;

/**
 * CheesyDriveHelper implements the calculations used in CheesyDrive for teleop control. Returns a DriveSignal for the motor output
 */
public class CheesyDriveHelper {
	private double mOldWheel, mQuickStopAccumulator;
	private boolean mInitialBrake;
	private double mOldThrottle, mBrakeRate;

	public DriveSignal cheesyDrive(Commands commands, RobotState robotState) {
		double throttle = -robotState.leftStickInput.getY();
		double wheel = robotState.rightStickInput.getX();

		//Quickturn if right trigger is pressed
		boolean isQuickTurn = robotState.rightStickInput.getTriggerPressed();

		//Braking if left trigger is pressed
		boolean isBraking = robotState.leftStickInput.getTriggerPressed();

		double wheelNonLinearity;

		wheel = ChezyMath.handleDeadband(wheel, Constants.kDeadband);
		throttle = ChezyMath.handleDeadband(throttle, Constants.kDeadband);

		double negInertia = wheel - mOldWheel;
		mOldWheel = wheel;

		wheelNonLinearity = 0.5;

		//Applies a sin function that is scaled
		wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);

		double leftPwm, rightPwm, overPower;
		double sensitivity;

		double angularPower;
		double linearPower = remapThrottle(throttle);
		;

		//Negative inertia
		double negInertiaAccumulator = 0.0;
		double negInertiaScalar;

		if(wheel * negInertia > 0) {
			negInertiaScalar = 2.5;
		} else {
			if(Math.abs(wheel) > 0.65) {
				negInertiaScalar = 5.0;
			} else {
				negInertiaScalar = 3.0;
			}
		}

		sensitivity = Constants.kDriveSensitivity;

		//neginertia is difference in wheel
		double negInertiaPower = negInertia * negInertiaScalar;
		negInertiaAccumulator += negInertiaPower;

		//possible source of occasional overturn
		wheel = wheel + negInertiaAccumulator;

		//Handle braking
		if(isBraking) {
			//Set up braking rates for linear deceleration in a set amount of time
			if(mInitialBrake) {
				mInitialBrake = false;
				//Old throttle initially set to throttle
				mOldThrottle = linearPower;
				//Braking rate set
				mBrakeRate = mOldThrottle / Constants.kCyclesUntilStop;
			}

			//If braking is not complete, decrease by the brake rate
			if(Math.abs(mOldThrottle) >= Math.abs(mBrakeRate)) {
				//reduce throttle
				mOldThrottle -= mBrakeRate;
				linearPower = mOldThrottle;
			} else {
				linearPower = 0;
			}
		} else {
			mInitialBrake = true;
		}

		//Quickturn
		if(isQuickTurn) {
			if(Math.abs(robotState.rightStickInput.getX()) < Constants.kQuickTurnSensitivityThreshold) {
				sensitivity = Constants.kPreciseQuickTurnSensitivity;
			} else {
				sensitivity = Constants.kQuickTurnSensitivity;
			}

			angularPower = wheel * sensitivity;

			//Can be tuned
			double alpha = Constants.kAlpha;
			mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator + alpha * angularPower * 5.5;

			overPower = 1.0;
		} else {
			overPower = 0.0;

			//Sets turn amount
			angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumulator;

			if(mQuickStopAccumulator > Constants.kQuickStopAccumulatorDecreaseThreshold) {
				mQuickStopAccumulator -= Constants.kQuickStopAccumulatorDecreaseRate;
			} else if(mQuickStopAccumulator < -Constants.kQuickStopAccumulatorDecreaseThreshold) {
				mQuickStopAccumulator += Constants.kQuickStopAccumulatorDecreaseRate;
			} else {
				mQuickStopAccumulator = 0.0;
			}
		}

		rightPwm = leftPwm = linearPower;
		leftPwm += angularPower;
		rightPwm -= angularPower;

		if(leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if(rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if(leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if(rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}

		DriveSignal mSignal = DriveSignal.getNeutralSignal();

		mSignal.leftMotor.setPercentOutput(Math.min(0.7, leftPwm));
		mSignal.rightMotor.setPercentOutput(Math.min(0.7, rightPwm));
		return mSignal;
	}

	/**
	 * Throttle tuning functions
	 */
	public double remapThrottle(double initialThrottle) {
		double x = Math.abs(initialThrottle);
		switch(Constants.kDriverName) {
			case ERIC:
				x = Math.signum(initialThrottle) * x;
				break;
		}
		return x;
	}

	/**
	 * Limits the given input to the given magnitude.
	 */
	public double limit(double v, double limit) {
		return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
	}
}