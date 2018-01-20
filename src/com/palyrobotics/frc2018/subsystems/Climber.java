package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.robot.Robot;
import com.palyrobotics.frc2018.util.ClimberSignal;

public class Climber extends Subsystem {

	public enum Side {
		NOT_SET,
		LEFT,
		RIGHT
	}
	
	public enum MotionSubstate {
		MOVING,
		LOCKED
	}

	public enum LockState {
		UNLOCKED,
		LOCKED
	}

	private Side mSide;
	private MotionSubstate mMotionStatus;
	private LockState mLock;
	private ClimberSignal mSignal;

	private static Climber instance = new Climber();

	public static Climber getInstance() {
		return instance;
	}
	
	private Climber() {
		super("Climber");
		mSide = Side.NOT_SET;
		mMotionStatus = MotionSubstate.LOCKED;
		mLock = LockState.UNLOCKED;
	}

	@Override
	public void update(Commands commands, RobotState robotState) {
		mMotionStatus = commands.wantedClimbMovement;
		mSide = commands.wantedClimbSide;
		mLock = commands.wantedLockState;

		double motorOutput;
		boolean brake;
		boolean lock;

		if (this.mMotionStatus == MotionSubstate.MOVING) {
			motorOutput = Robot.getRobotState().operatorStickInput.getY();
			brake = false;
		} else {
			motorOutput = 0;
			brake = true;
		}

		if (this.mLock == LockState.UNLOCKED) {
			lock = false;
		} else {
			lock = true;
		}

		double mLeftMotor = 0.0, mRightMotor = 0.0;
		boolean leftBrake = true, rightBrake = true, leftLock = false, rightLock = false;

		if (this.getSide() == Climber.Side.LEFT) {
			mLeftMotor = motorOutput;
			leftBrake = brake;
			leftLock = lock;
		}
		else if (this.getSide() == Side.RIGHT) {
			mRightMotor = motorOutput;
			rightBrake = brake;
			rightLock = lock;
		}
		mSignal = new ClimberSignal(mLeftMotor, mRightMotor, leftBrake, rightBrake, leftLock, rightLock);
	}

	public ClimberSignal getSignal() {
		return mSignal;
	}

	public Side getSide() {
		return mSide;
	}

	public MotionSubstate getMotionSubstate() {
		return mMotionStatus;
	}

	public LockState getLock() {
		return mLock;
	}
}