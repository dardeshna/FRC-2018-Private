package com.palyrobotics.frc2018.util.archive;

import com.palyrobotics.frc2018.util.JoystickInput;

public class XboxInput extends JoystickInput {
    public double leftX, leftY, rightX, rightY, leftTrigger, rightTrigger;
    public boolean leftTriggerPressed, rightTriggerPressed, buttonA, buttonB, buttonX, buttonY, buttonStart, buttonBack, dPadUp;

    public void update(XboxController x) {
        this.leftX = x.getLeftX();
        this.leftY = x.getLeftY();
        this.rightX = x.getRightX();
        this.rightY = x.getRightY();
        this.leftTrigger = x.getLeftTrigger();
        this.rightTrigger = x.getRightTrigger();
        this.leftTriggerPressed = x.getLeftTriggerPressed();
        this.rightTriggerPressed = x.getRightTriggerPressed();
        this.buttonA = x.getButtonA();
        this.buttonB = x.getButtonB();
        this.buttonX = x.getButtonX();
        this.buttonY = x.getButtonY();
        this.buttonStart = x.getButtonStart();
        this.buttonBack = x.getButtonBack();
        this.dPadUp = x.getDpadUp();
    }
}
