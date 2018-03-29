package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.util.ClimberSignal;

public class Climber extends Subsystem {

	public enum MotionSubstate {
		MOVING, LOCKED
	}

	public enum LockState {
		UNLOCKED, LOCKED
	}

	private MotionSubstate mMotionStatus;
	private LockState mLock;
	private ClimberSignal mSignal = ClimberSignal.getNeutralSignal();


	private static Climber instance = new Climber();

	public static Climber getInstance() {
		return instance;
	}

	protected Climber() {
		super("Climber");
		mMotionStatus = MotionSubstate.LOCKED;
		mLock = LockState.UNLOCKED;
	}

	@Override
	public void update(Commands commands, RobotState robotState) {
		mMotionStatus = commands.wantedClimbMovement;
		mLock = commands.wantedLockState;

		double motorOutput;
		boolean brake;
		boolean lock;

		if(this.mMotionStatus == MotionSubstate.MOVING) {
			if(Constants.operatorXBoxController) {
				motorOutput = robotState.operatorXboxControllerInput.getLeftY();
			} else {
				motorOutput = robotState.climberStickInput.getY();
			}

			brake = false;
		} else {
			motorOutput = 0;
			brake = true;
		}

		if(this.mLock == LockState.UNLOCKED) {
			lock = false;
		} else {
			lock = true;
		}

		mSignal = new ClimberSignal(motorOutput, brake, lock);
	}

	@Override
	public void stop() {
		mSignal = ClimberSignal.getNeutralSignal();
	}

	public ClimberSignal getSignal() {
		return mSignal;
	}

	public MotionSubstate getMotionSubstate() {
		return mMotionStatus;
	}

	public LockState getLock() {
		return mLock;
	}
}