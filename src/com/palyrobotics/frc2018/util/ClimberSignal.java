package com.palyrobotics.frc2018.util;

public class ClimberSignal {

	public double leftVelocity;
	public double rightVelocity;

	public boolean leftBrake, rightBrake, leftLatchLock, rightLatchLock;

	public ClimberSignal(double leftVelocity, double rightVelocity, boolean leftBrake, boolean rightBrake, boolean leftLatchLock, boolean rightLatchLock) {
		this.leftBrake = leftBrake;
		this.rightBrake = rightBrake;
		this.leftLatchLock = leftLatchLock;
		this.rightLatchLock = rightLatchLock;
		this.leftVelocity = leftVelocity;
		this.rightVelocity = rightVelocity;
	}
}