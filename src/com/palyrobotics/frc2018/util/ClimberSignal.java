package com.palyrobotics.frc2018.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberSignal {

    public double leftVelocity;
    public double rightVelocity;

    public boolean leftBrake, rightBrake, latchLock, latchLockRight;

    public ClimberSignal(double leftVelocity, double rightVelocity, boolean leftBrake,
                         boolean rightBrake, boolean latchLock, boolean rightLatchLock) {
        this.leftBrake = leftBrake;
        this.rightBrake = rightBrake;
        this.latchLock = latchLock;
        this.latchLockRight = rightLatchLock;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
    }
}
