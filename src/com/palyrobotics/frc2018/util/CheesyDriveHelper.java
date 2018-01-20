package com.palyrobotics.frc2018.util;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.util.ChezyMath;

/**
 * CheesyDriveHelper implements the calculations used in CheesyDrive for teleop control.
 * Returns a DriveSignal for the motor output
 */
public class CheesyDriveHelper {
	private double mOldWheel, mQuickStopAccumulator;
	private boolean mInitialBrake;
	private double mOldThrottle, mBrakeRate;
	private final double kWheelStickDeadband = 0.02;
	private final double kThrottleStickDeadband = 0.02;

	public DriveSignal cheesyDrive(Commands commands, RobotState robotState) {
		double throttle = -robotState.leftStickInput.y;
		double wheel = robotState.rightStickInput.x;

		//Quickturn if right trigger is pressed
		boolean isQuickTurn = robotState.rightStickInput.triggerPressed;

		//Braking if left trigger is pressed
		boolean isBraking = robotState.leftStickInput.triggerPressed;

		double wheelNonLinearity;

		wheel = ChezyMath.handleDeadband(wheel, kWheelStickDeadband);
		throttle = ChezyMath.handleDeadband(throttle, kThrottleStickDeadband);

		double negInertia = wheel - mOldWheel;
		mOldWheel = wheel;

		wheelNonLinearity = 0.5;
		
		//Applies a sin function that is scaled 
		wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
				/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
				/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
				/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);

		double leftPwm, rightPwm, overPower;
		double sensitivity;
		
		double angularPower;
		double linearPower = remapThrottle(throttle);;

		//Negative inertia
		double negInertiaAccumulator = 0.0;
		double negInertiaScalar;
		
		if (wheel * negInertia > 0) {
			negInertiaScalar = 2.5;
		} else {
			if (Math.abs(wheel) > 0.65) {
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
		
		//limit between [-1, 1]
		if (negInertiaAccumulator > 1) {
			negInertiaAccumulator -= 1;
		} else if (negInertiaAccumulator < -1) {
			negInertiaAccumulator += 1;
		} else {
			negInertiaAccumulator = 0;
		}

		//Handle braking
		if(isBraking) {
			//Set up braking rates for linear deceleration in a set amount of time
			if(mInitialBrake) {
				mInitialBrake = false;
				//Old throttle initially set to throttle
				mOldThrottle = linearPower;
				//Braking rate set
				mBrakeRate = mOldThrottle/Constants.kCyclesUntilStop;
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
		if (isQuickTurn) {
			//Can be tuned
			double alpha = Constants.kAlpha;
			mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator
					+ alpha * limit(wheel, 1.0) * 5;
			
			overPower = 1.0;
			
			if(Math.abs(robotState.rightStickInput.x) < Constants.kQuickTurnSensitivityThreshold) {
				sensitivity = Constants.kPreciseQuickTurnSensitivity;
			} else {
				sensitivity = Constants.kQuickTurnSensitivity;
			}
			
			angularPower = wheel * sensitivity;
			
		} else {
			overPower = 0.0;

			//Sets turn amount
			angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumulator;

			if (mQuickStopAccumulator > Constants.kQuickStopAccumulatorDecreaseThreshold) {
				mQuickStopAccumulator -= Constants.kQuickStopAccumulatorDecreaseRate;
			} else if (mQuickStopAccumulator < -Constants.kQuickStopAccumulatorDecreaseThreshold) {
				mQuickStopAccumulator += Constants.kQuickStopAccumulatorDecreaseRate;
			} else {
				mQuickStopAccumulator = 0.0;
			}
		}
		
		rightPwm = leftPwm = linearPower;
		leftPwm += angularPower;
		rightPwm -= angularPower;

		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}
		
		DriveSignal mSignal = DriveSignal.getNeutralSignal();
		mSignal.leftMotor.setPercentOutput(leftPwm);
		mSignal.rightMotor.setPercentOutput(rightPwm);
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