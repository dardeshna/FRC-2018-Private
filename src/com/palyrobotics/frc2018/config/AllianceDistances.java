package com.palyrobotics.frc2018.config;

/**
 * Contains the field distances for an alliance
 * @author Jason
 */
public class AllianceDistances {
    //Base line
    public double kBaseLineDistanceInches = 122.0;

    //Switch
    public double kRightSwitchX = 140.0;
    public double kRightSwitchY = 84.5;

    public double kLeftSwitchX = 140.25;
    public double kLeftSwitchY = 85.75;

    public double kPyramidFromRightY = 139.5;

    //Scales
    public double kLeftScaleX = 298.0;
    public double kLeftScaleY = 73.0;

    public double kRightScaleX = 298.0;
    public double kRightScaleY = 73.0;

    public double kPyramidWidth = 45.0;
    public double kPyramidLength = 40.0;

    //Offsets
    public double kLeftCornerOffset = 29.0;
    public double kRightCornerOffset = 29.75;

    //Length from left field wall to right edge of the exchange zone
    public double kLeftToCenterY = 149.25;

    //Distance to the somewhat arbitrary line between the scale and the switch along which the robot drives
    public double kScaleSwitchMidlineX = 233.0;

    public double kLeftPlatformY = 95.25;
    public double kRightPlatformY = 95.25;

    //Self explanatory
    public double kFieldWidth = 324.0;
    public double kSwitchPlateWidth = 41.75;
    public double kSwitchPlateLength = 56.0;
    public double kScalePlateWidth = 36.5;
    public double kScalePlateLength = 48.0;
}
