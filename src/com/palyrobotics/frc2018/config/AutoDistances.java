package com.palyrobotics.frc2018.config;

/**
 * Created by Eric on 2/12/18
 */
public class AutoDistances {
	//Base line
	public static double kRedBaseLineDistanceInches;
	public static double kBlueBaseLineDistanceInches;

	//Switches
	public static double kRedRightSwitchX;
	public static double kRedRightSwitchY;
	
	public static double kBlueRightSwitchX;
	public static double kBlueRightSwitchY;

	public static double kRedLeftSwitchX;
	public static double kRedLeftSwitchY;
	
	public static double kBlueLeftSwitchX;
	public static double kBlueLeftSwitchY;

    public static double pyramidStackX;
	public static double pyramidStackY;

	//Scales
	public static double kBlueLeftScaleX;
	public static double kBlueLeftScaleY;

	public static double kBlueRightScaleX;
	public static double kBlueRightScaleY;

	public static double kRedLeftScaleX;
	public static double kRedLeftScaleY;

	public static double kRedRightScaleX;
	public static double kRedRightScaleY;

	//Offsets
	public static double kRedLeftCornerOffset;
	public static double kRedRightCornerOffset;

	public static double kBlueLeftCornerOffset;
	public static double kBlueRightCornerOffset;

	//Self explanatory
	public static double kFieldWidth;

	//Length from left field wall to right edge of the exchange zone
	public static double kBlueLeftToCenterY;
	public static double kRedLeftToCenterY;

	//Distance to the somewhat arbitrary line between the scale and the switch along which the robot drives
	public static double kBlueScaleSwitchMidlineX;
	public static double kRedScaleSwitchMidlineX;
}
