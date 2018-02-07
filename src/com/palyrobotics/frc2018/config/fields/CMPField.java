package com.palyrobotics.frc2018.config.fields;

import com.palyrobotics.frc2018.config.AutoDistances;

public class CMPField {
    public static void configureFieldMeasurements() {
        //Base line
        AutoDistances.kRedBaseLineDistanceInches = 0;
        AutoDistances.kBlueBaseLineDistanceInches = 0;

        //Switches
        AutoDistances.kRedRightSwitchX = 0;
        AutoDistances.kRedRightSwitchY = 0;

        AutoDistances.kBlueRightSwitchX = 0;
        AutoDistances.kBlueRightSwitchY = 0;

        AutoDistances.kRedLeftSwitchX = 0;
        AutoDistances.kRedLeftSwitchY = 0;

        AutoDistances.kBlueLeftSwitchX = 0;
        AutoDistances.kBlueLeftSwitchY = 0;

        //Scales
        AutoDistances.kBlueLeftScaleX = 0;
        AutoDistances.kBlueLeftScaleY = 0;

        AutoDistances.kBlueRightScaleX = 0;
        AutoDistances.kBlueRightScaleY = 0;

        AutoDistances.kRedLeftScaleX = 0;
        AutoDistances.kRedLeftScaleY = 0;

        AutoDistances.kRedRightScaleX = 0;
        AutoDistances.kRedRightScaleY = 0;

        //Offsets
        AutoDistances.kRedLeftCornerOffset = 0;
        AutoDistances.kRedRightCornerOffset = 0;

        AutoDistances.kBlueLeftCornerOffset = 0;
        AutoDistances.kBlueRightCornerOffset = 0;

        //Self explanatory
        AutoDistances.kFieldWidth = 0;

        //Length from left field wall to right edge of the exchange zone
        AutoDistances.kBlueLeftToCenterY = 0;
        AutoDistances.kRedLeftToCenterY = 0;

        //Distance to the somewhat arbitrary line between the scale and the switch along which the robot drives
        AutoDistances.kBlueScaleSwitchMidlineX = 0;
        AutoDistances.kRedScaleSwitchMidlineX = 0;
    }
}
