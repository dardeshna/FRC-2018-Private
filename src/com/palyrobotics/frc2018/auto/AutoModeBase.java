package com.palyrobotics.frc2018.auto;

import com.palyrobotics.frc2018.behavior.Routine;

public abstract class AutoModeBase {
	protected boolean active = false;

	public abstract String toString();

	//Will be run before the routine is taken
	public abstract void prestart();
	
	public enum Alliance {
		RED,
		BLUE
	}

	public enum StartingPosition {
		LEFT,
		CENTER,
		RIGHT
	}

	public enum Decision {
		NEVER,
		LEFT,
		RIGHT,
		BOTH
	}

	public enum Priority {
		SCALE,
		SWITCH
	}
	
	public static Alliance mAlliance = null;
	public static StartingPosition mStartingPosition = null;
	public static Decision mScaleDecision = null;
	public static Decision mSwitchDecision = null;
	public static Priority mPriority = null;

	public abstract Routine getRoutine();

	public void stop() {
		active = false;
	}

	public boolean isActive() {
		return active;
	}
}