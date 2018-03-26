package com.palyrobotics.frc2018.config;

import java.io.File;
import java.util.logging.Level;

import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.JSONFormatter;

/**
 * Created by Eric on 2/12/18
 */
public class AutoDistances {
	//Base line
	public static double kRedBaseLineDistanceInches = 122.0;
    public static double kBlueBaseLineDistanceInches = 122.125;
    
	//Switches
    public static double kRedRightSwitchX = 140.0;
    public static double kRedRightSwitchY = 84.5;
    
    public static double kBlueRightSwitchX = 140.0;
    public static double kBlueRightSwitchY = 85.5;
    
    public static double kRedLeftSwitchX = 140.25;
    public static double kRedLeftSwitchY = 85.75;
    
    public static double kBlueLeftSwitchX = 140.0;
    public static double kBlueLeftSwitchY = 84.75;
    
    public static double kBluePyramidFromRightY = 139.25;
    public static double kRedPyramidFromRightY = 139.5;
    
	//Scales
    public static double kBlueLeftScaleX = 298.0;
    public static double kBlueLeftScaleY = 73.0;
    
    public static double kBlueRightScaleX = 298.0;
    public static double kBlueRightScaleY = 73.0;
    
    public static double kRedLeftScaleX = 299.0;
    public static double kRedLeftScaleY = 73.0;
    
    public static double kRedRightScaleX = 299.0;
    public static double kRedRightScaleY = 73.0;
    
    public static double kBluePyramidWidth = 45.0;
    public static double kBluePyramidLength = 40.0;
    
    public static double kRedPyramidWidth = 44.75;
    public static double kRedPyramidLength = 40.25;
    
	//Offsets
    public static double kRedLeftCornerOffset = 29.0;
    public static double kRedRightCornerOffset = 29.75;
    
    public static double kBlueLeftCornerOffset = 28.5;
    public static double kBlueRightCornerOffset = 30.0;
    
	//Self explanatory
    public static double kFieldWidth = 324.0;
    public static double kSwitchPlateWidth = 41.75;
    public static double kSwitchPlateLength = 56.0;
    public static double kScalePlateWidth = 36.5;
    public static double kScalePlateLength = 48.0;
    
	//Length from left field wall to right edge of the exchange zone
    public static double kBlueLeftToCenterY = 149.25;
    public static double kRedLeftToCenterY = 151.125;
    
	//Distance to the somewhat arbitrary line between the scale and the switch along which the robot drives
    public static double kBlueScaleSwitchMidlineX = 233.0;
    public static double kRedScaleSwitchMidlineX = 233.0;

    public static double kBlueLeftPlatformY = 95.25;
    public static double kBlueRightPlatformY = 95.25;
    public static double kRedLeftPlatformY = 95.25;
    public static double kRedRightPlatformY = 95.25;

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
		if (field == null) { // If the field file doesn't exist, keep default values
			Logger.getInstance().logRobotThread(Level.FINE, "Field json file not found");
			return;
		}
		
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
		kBluePyramidWidth = getDoubleValue("kBluePyramidWidth");
		kBluePyramidLength = getDoubleValue("kBluePyramidLength");
		kRedPyramidWidth = getDoubleValue("kRedPyramidWidth");
		kRedPyramidLength = getDoubleValue("kRedPyramidLength");
		kRedLeftCornerOffset = getDoubleValue("kRedLeftCornerOffset");
		kRedRightCornerOffset = getDoubleValue("kRedRightCornerOffset");
		kBlueLeftCornerOffset = getDoubleValue("kBlueLeftCornerOffset");
		kBlueRightCornerOffset = getDoubleValue("kBlueRightCornerOffset");
		kFieldWidth = getDoubleValue("kFieldWidth");
		kBlueLeftToCenterY = getDoubleValue("kBlueLeftToCenterY");
		kRedLeftToCenterY = getDoubleValue("kRedLeftToCenterY");
		kBlueScaleSwitchMidlineX = getDoubleValue("kBlueScaleSwitchMidlineX");
		kRedScaleSwitchMidlineX = getDoubleValue("kRedScaleSwitchMidlineX");
		kSwitchPlateWidth = getDoubleValue("kSwitchPlateWidth");
		kSwitchPlateLength = getDoubleValue("kSwitchPlateLength");
		kScalePlateWidth = getDoubleValue("kScalePlateWidth");
		kScalePlateLength = getDoubleValue("kScalePlateLength");
		kBlueLeftPlatformY = getDoubleValue("kBlueLeftPlatformY");
		kBlueRightPlatformY = getDoubleValue("kBlueRightPlatformY");
		kBlueLeftPlatformY = getDoubleValue("kBlueLeftPlatformY");
		kRedLeftPlatformY = getDoubleValue("kRedLeftPlatformY");
		kRedRightPlatformY = getDoubleValue("kRedRightPlatformY");
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
