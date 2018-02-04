package com.palyrobotics.frc2018.auto;

import java.util.logging.Level;

import com.palyrobotics.frc2018.util.logger.Logger;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoFMS {

    public enum Side {
        LEFT,
        RIGHT
    }

    protected AutoFMS() {

    }

    private static AutoFMS instance_ = new AutoFMS();

    public static AutoFMS getInstance() {
        return instance_;
    }

    public Side getSwitchSide() {
        String dataString = DriverStation.getInstance().getGameSpecificMessage();
        if(String.valueOf(dataString.charAt(0)).equals("L")) {
            return Side.LEFT;
        } else if(String.valueOf(dataString.charAt(0)).equals("R")) {
            return Side.RIGHT;
        } else {
            Logger.getInstance().logRobotThread(Level.SEVERE, "Failed to receive switch side from FMS!");
            return null;
        }
    }

    public Side getScaleSide() {
        String dataString = DriverStation.getInstance().getGameSpecificMessage();
        if(String.valueOf(dataString.charAt(1)).equals("L")) {
            return Side.LEFT;
        } else if(String.valueOf(dataString.charAt(1)).equals("R")) {
            return Side.RIGHT;
        } else {
            Logger.getInstance().logRobotThread(Level.SEVERE, "Failed to receive scale side from FMS!");
            return null;
        }
    }
}
