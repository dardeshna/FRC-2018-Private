package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.behavior.Routine;

public class LeftStartRightScaleRightSwitchAutoMode extends AutoModeBase {

    public LeftStartRightScaleRightSwitchAutoMode(Alliance alliance) {
        super(alliance);
    }

    @Override
    public String toString() {
        return null;
    }

    @Override
    public void prestart() {

    }

    @Override
    public Routine getRoutine() {
        return null;
    }

    @Override
    public String getKey() {
        return mAlliance + " LEFT SCALE RIGHT SWITCH RIGHT";
    }
}
