package com.palyrobotics.frc2018.auto;

import com.palyrobotics.frc2018.behavior.Routine;

public abstract class AutoModeBase {
	protected boolean active = false;

	public abstract String toString();

	//Will be run before the routine is taken
	public abstract void prestart();

	public AutoModeBase() {}

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

	public enum SecondSideDecision {
	    NEVER,
	    OPPOSITE,
        SAME,
        BOTH
    }

	public enum Priority {
		SCALE,
		SWITCH
	}
	
	// To set the auto mode, set these variables in code!
	public static Alliance mAlliance = Alliance.RED;
	public static StartingPosition mStartingPosition = StartingPosition.CENTER;
	public static Decision mScaleDecision = Decision.NEVER;
	public static Decision mSwitchDecision = Decision.BOTH;
    public static SecondSideDecision mSecondScaleSideDecision = SecondSideDecision.NEVER;
    public static SecondSideDecision mSecondSwitchSideDecision = SecondSideDecision.SAME;
	public static Priority mPriority = Priority.SWITCH;
	public static Priority mSecondCubePriority = Priority.SWITCH;
	public static boolean mMultiCube = true;

	public abstract Routine getRoutine();

	public void stop() {
		active = false;
	}

	public boolean isActive() {
		return active;
	}
	
	public abstract String getKey();
}