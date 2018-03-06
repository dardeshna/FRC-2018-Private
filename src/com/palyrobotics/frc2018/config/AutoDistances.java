package com.palyrobotics.frc2018.config;

import java.io.File;

import com.palyrobotics.frc2018.util.JSONFormatter;

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

    public static double kBluePyramidFromRightY;
	public static double kRedPyramidFromRightY;

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
	
	public static void updateAutoDistances() {
		loadField();
		setAutoDistances();
	}
	
	private static File field;
	
	private static void loadField() { // Make sure to ant deploy constants to roboRIO
		switch (Constants.kFieldName) {
		case AZN:
			field = JSONFormatter.loadFileDirectory("constants/fields", "AZNField.json");
			break;
		case AZN_PRACTICE:
			field = JSONFormatter.loadFileDirectory("constants/fields", "AZNPracticeField.json");
			break;
		case CMP:
			field = JSONFormatter.loadFileDirectory("constants/fields", "CMPField.json");
			break;
		case CMP_PRACTICE:
			field = JSONFormatter.loadFileDirectory("constants/fields", "CMPPracticeField.json");
			break;
		case SVR:
			field = JSONFormatter.loadFileDirectory("constants/fields", "SVRField.json");
			break;
		case SVR_PRACTICE:
			field = JSONFormatter.loadFileDirectory("constants/fields", "SVRPracticeField.json");
			break;
		case TEAM_254:
			field = JSONFormatter.loadFileDirectory("constants/fields", "Team254Field.json");
			break;
		case TEAM_8:
			field = JSONFormatter.loadFileDirectory("constants/fields", "Team8Field.json");
			break;
		}
	}
	
	private static void setAutoDistances() { // run after loading chosen field
		kRedBaseLineDistanceInches = getDoubleValue("kRedBaseLineDistanceInches");
		kBlueBaseLineDistanceInches = getDoubleValue("kBlueBaseLineDistanceInches");
		kRedRightSwitchX = getDoubleValue("kRedRightSwitchX");
		kRedRightSwitchY = getDoubleValue("kRedRightSwitchY");
		kBlueRightSwitchX = getDoubleValue("kBlueRightSwitchX");
		kBlueRightSwitchY = getDoubleValue("kBlueRightSwitchY");
		kRedLeftSwitchX = getDoubleValue("kRedLeftSwitchX");
		kRedLeftSwitchY = getDoubleValue("kRedLeftSwitchY");
		kBlueLeftSwitchX = getDoubleValue("kBlueLeftSwitchX");
		kBlueLeftSwitchY = getDoubleValue("kBlueLeftSwitchY");
		kBluePyramidFromRightY = getDoubleValue("kBluePyramidFromRightY");
		kRedPyramidFromRightY = getDoubleValue("kRedPyramidFromRightY");
		kBlueLeftScaleX = getDoubleValue("kBlueLeftScaleX");
		kBlueLeftScaleY = getDoubleValue("kBlueLeftScaleY");
		kBlueRightScaleX = getDoubleValue("kBlueRightScaleX");
		kBlueRightScaleY = getDoubleValue("kBlueRightScaleY");
		kRedLeftScaleX = getDoubleValue("kRedLeftScaleX");
		kRedLeftScaleY = getDoubleValue("kRedLeftScaleY");
		kRedRightScaleX = getDoubleValue("kRedRightScaleX");
		kRedRightScaleY = getDoubleValue("kRedRightScaleY");
		kRedLeftCornerOffset = getDoubleValue("kRedLeftCornerOffset");
		kRedRightCornerOffset = getDoubleValue("kRedRightCornerOffset");
		kBlueLeftCornerOffset = getDoubleValue("kBlueLeftCornerOffset");
		kBlueRightCornerOffset = getDoubleValue("kBlueRightCornerOffset");
		kFieldWidth = getDoubleValue("kFieldWidth");
		kBlueLeftToCenterY = getDoubleValue("kBlueLeftToCenterY");
		kRedLeftToCenterY = getDoubleValue("kRedLeftToCenterY");
		kBlueScaleSwitchMidlineX = getDoubleValue("kBlueScaleSwitchMidlineX");
		kRedScaleSwitchMidlineX = getDoubleValue("kRedScaleSwitchMidlineX");
	}
	
	/**
	 * Return value of given key in the given field
	 * @param key
	 * @return
	 */
	private static Double getDoubleValue(String key) {
		Object value = JSONFormatter.getValueInFile(field, key);
		return (Double) value;
	}
}
