package com.palyrobotics.frc2018.util;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Class to store Joystick input
 * @author Nihar
 */
public class JoystickInput {

    public double x,y;
    public boolean triggerPressed, button1, button2,
            button3, button4, button5, button6,
            button7, button8, button9, button10,
            button11, button12, button13, button14;

    @Override
    public String toString() {
        return "Joystick X: "+this.x+" Y: "+ this.y;
    }

    public void update(Joystick j) {
        x = j.getX();
        y = j.getY();
        triggerPressed = j.getTrigger();
        button1 = j.getRawButton(1);
        button2 = j.getRawButton(2);
        button3 = j.getRawButton(3);
        button4 = j.getRawButton(4);
        button5 = j.getRawButton(5);
        button6 = j.getRawButton(6);
        button7 = j.getRawButton(7);
        button8 = j.getRawButton(8);
        button9 = j.getRawButton(9);
        button10 = j.getRawButton(10);
        button11 = j.getRawButton(11);
        button12 = j.getRawButton(12);
        button13 = j.getRawButton(13);
        button14 = j.getRawButton(14);
    }
}
