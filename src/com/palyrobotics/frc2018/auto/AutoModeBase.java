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
	
	// To set the auto mode, set these variables in code!
	public static Alliance mAlliance = Alliance.BLUE;
	public static StartingPosition mStartingPosition = StartingPosition.CENTER;
	public static Decision mScaleDecision = Decision.RIGHT;
	public static Decision mSwitchDecision = Decision.RIGHT;
	public static Priority mPriority = Priority.SCALE;

	public abstract Routine getRoutine();

	public void stop() {
		active = false;
	}

	public boolean isActive() {
		return active;
	}
	
	public String getKey() {
		return "";
	}
}