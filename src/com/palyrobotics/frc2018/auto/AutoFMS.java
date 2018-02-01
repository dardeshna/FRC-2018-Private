package com.palyrobotics.frc2018.auto;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoFMS {

    public enum Side {
        LEFT,
        RIGHT
    }

    protected AutoFMS() {

    }

    public static AutoFMS instance_ = new AutoFMS();

    public static AutoFMS getInstance() {
        return instance_;
    }

    public Side getSwitchSide() {
        String dataString = DriverStation.getInstance().getGameSpecificMessage();
        if (String.valueOf(dataString.charAt(0)).equals("L")) {
            return Side.LEFT;
        } else {
            return Side.RIGHT;
        }
    }

    public Side getScaleSide() {
        String dataString = DriverStation.getInstance().getGameSpecificMessage();
        if (String.valueOf(dataString.charAt(1)).equals("L")) {
            return Side.LEFT;
        } else {
            return Side.RIGHT;
        }
    }
}
