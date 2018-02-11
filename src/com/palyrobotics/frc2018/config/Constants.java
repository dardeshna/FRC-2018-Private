package com.palyrobotics.frc2018.config;

public class Constants {
	public enum RobotName {
		FORSETI
	}

	public enum DriverName {
		ERIC
	}

	public enum OperatorName {
		JACOB
	}

	public enum FieldName {
		//we goin to cmp bois
		TEAM_8, TEAM_254, AZN_PRACTICE, AZN, SVR_PRACTICE, SVR, CMP_PRACTICE, CMP
	}

	//Initialization constants
	public static final RobotName kRobotName = RobotName.FORSETI;
	public static final DriverName kDriverName = DriverName.ERIC;
	public static final OperatorName kOperatorName = OperatorName.JACOB;
	public static final FieldName kFieldName = FieldName.TEAM_8;

	//Android app information
	public static final String kPackageName = "com.frc8.team8vision";
	public static final String kActivityName = "MainActivity";
	public static final String kVisionDataFileName = "data.json";
	public static final String kVisionVideoFileName = "video.json";
	public static final int kAndroidConnectionUpdateRate = 5; //Update rate in milliseconds
	public static final int kAndroidDataSocketUpdateRate = 100;
	public static final int kAndroidVisionSocketUpdateRate = 10;
	public static final int kMJPEGVisionSocketUpdateRate = 20;
	public static final int kVisionDataPort = 8008;
	public static final int kVideoPort = 8009;
	public static final int kMJPEGServerSocketPort = 1180;

	/**
	 * Cheesy Drive Constants Set by DriverProfiles
	 */
	//Deadband for joysticks
	public static double kDeadband;
	//Threshold for quickturn sensitivity change
	public static double kQuickTurnSensitivityThreshold;
	//Sensitivities for how fast non-quickturn turning is
	public static double kDriveSensitivity;
	//Sensitivities for quickturn
	public static double kQuickTurnSensitivity;
	public static double kPreciseQuickTurnSensitivity;
	//The rate at which the QuickStopAccumulator will decrease
	public static double kQuickStopAccumulatorDecreaseRate;
	//The value at which the QuickStopAccumulator will begin to decrease
	public static double kQuickStopAccumulatorDecreaseThreshold;
	public static double kNegativeInertiaScalar;
	//How much the QuickStopAccumulator is affected by the wheel
	//(1-alpha) is how much the QuickStopAccumulator is affected by the previous QuickStopAccumulator
	//Range: (0, 1)
	public static double kAlpha;
	public static double kCyclesUntilStop;

	/**
	 * Elevator Constants
	 */
	public static final double kTopBottomEncoderDifference = 1000;
	public static final double kNominalUpwardsOutput = 0.1;
	public static final double kElevatorTopPositionInches = 75;
	public static final double kElevatorSwitchPositionInches = 30;
	public static final double kElevatorBottomPositionInches = 0;

	/*
	 * Control loop constants for both robots
	 */
	public static final double kTurnInPlacePower = 0.17; //for bang bang
	public static final double kCalibratePower = -0.4;
	public static final double kDriveMaxClosedLoopOutput = 8.0;
	public static final double kElevatorMaxClosedLoopOutput = 0.666;

	/**
	 * Unit conversions for Talons
	 */
	public static final double kDriveTicksPerInch = 360 / (3.95 * Math.PI);
	public static final double kElevatorTicksPerInch = 360 / (3.95 * Math.PI); //TODO: Tune this!
	public static final double kDriveInchesPerDegree = 0.99 * 21.5 / 90;
	public static final double kDriveSpeedUnitConversion = 360 / (3.95 * Math.PI * 10);

	/**
	 * Physical robot Constants
	 */
	public static final double kRobotWidthInches = 27.0;
	public static final double kRobotLengthInches = 32.0;

	/**
	 * Tolerances
	 */
	public static final double kAcceptableDrivePositionError = 15;
	public static final double kAcceptableDriveVelocityError = 5;
	public static final double kAcceptableShortDrivePositionError = 1;
	public static final double kAcceptableShortDriveVelocityError = 3;
	public static final double kAcceptableTurnAngleError = 1;
	public static final double kAcceptableGyroZeroError = 3;
	public static final double kAcceptableEncoderZeroError = 10;

	public static final double kElevatorAcceptablePositionError = 0.01;
	public static final double kElevatorAcceptableVelocityError = 0.01;

	//Intake
	public static final double kIntakingMotorVelocity = 1.0;
	public static final double kExpellingMotorVelocity = -1.0;
	public static final double kExpelToScoreTime = 0.5;
	//TODO: Tune distance for ultrasound, determine variability
	public static final double kIntakeCloseSensorThreshold = 0.0;
	public static final double kIntakeFarSensorThreshold = 1.0;

	/*
	 * !!! End of editable Constants! !!!
	 **********************************************************************************
	 */
	public static final int kEndEditableArea = 0;

	/*
	 * ************************************ Forseti ELECTRONIC CONSTANTS ************************************
	 */
	//PDP
	public static final int kForsetiPDPDeviceID = 0;

	//DRIVETRAIN
	//PDP slots for drivetrain 0, 1, 2, 3, 12, 13
	public static final int kForsetiLeftDriveMasterDeviceID = 1;
	public static final int kForsetiLeftDriveSlaveDeviceID = 2;
	public static final int kForsetiLeftDriveOtherSlaveDeviceID = 3;
	public static final int kForsetiLeftDriveFrontMotorPDP = 0;
	public static final int kForsetiLeftDriveBackMotorPDP = 0;
	public static final int kForsetiLeftDriveThirdMotorPDP = 0;
	public static final int kForsetiRightDriveMasterDeviceID = 6;
	public static final int kForsetiRightDriveSlaveDeviceID = 5;
	public static final int kForsetiRightDriveOtherSlaveDeviceID = 4;
	public static final int kForsetiRightDriveFrontMotorPDP = 0;
	public static final int kForsetiRightDriveBackMotorPDP = 0;
	public static final int kForsetiRightDriveThirdMotorPDP = 0;

	//CLIMBER
	public static final int kForsetiClimberMotorLeftDeviceID = 1;
	public static final int kForsetiClimberMotorRightDeviceID = 2;

	public static final int kForsetiClimberLeftArmForward = 3;
	public static final int kForsetiClimberLeftArmBack = 3;

	public static final int kForsetiClimberRightArmForward = 3;
	public static final int kForsetiClimberRightArmBack = 3;

	public static final int kForsetiClimberLeftBrakeSolenoid = 4;
	public static final int kForsetiClimberRightBrakeSolenoid = 5;

	//ELEVATOR
	//PDP slots for elevator TBD (currently 7, 8, 9, 10)
	public static final int kForsetiElevatorMasterTalonID = 7;
	public static final int kForsetiElevatorSlaveTalonID = 9;
	public static final int kForsetiBottomElevatorHFXID = 8;
	public static final int kForsetiTopElevatorHFXID = 10;

	//INTAKE
	public static final int kForsetiIntakeMasterDeviceID = 7;
	public static final int kForsetiIntakeSlaveDeviceID = 8;
	public static final int kForsetiIntakeDistanceSensorID = 0;
	public static final int kForsetiIntakeUpDownSolenoidForwardID = 1;
	public static final int kForsetiIntakeUpDownSolenoidReverseID = 2;
	public static final int kForsetiIntakeOpenCloseOtherSolenoidForwardID = 0; //TODO: add actual ports
	public static final int kForsetiIntakeOpenCloseOtherSolenoidReverseID = 0;
	public static final int kForsetiIntakeOpenCloseSolenoidForwardID = 3;
	public static final int kForsetiIntakeOpenCloseSolenoidReverseID = 4;

	//!!! Physical constants
	public static final double kPlateWidth = 36.0;
	public static final double kPlateLength = 49.0;
	public static final double kSquareCubeLength = 13.0;

	//!!! Loop rate of normal Looper
	public static final double kNormalLoopsDt = 0.02;

	//Adaptive Pure Pursuit Controller
	public static final double kDriveWheelDiameterInches = 7.3;
	public static final double kTrackLengthInches = 8.265;
	public static final double kTrackWidthInches = 23.8;
	public static final double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
	public static final double kTrackScrubFactor = 0.9;
	public static final double kPathFollowingLookahead = 20.0;
	public static final double kPathFollowingMaxAccel = 5.0 * kDriveTicksPerInch;
	public static final double kPathFollowingMaxVel = 10.0 * kDriveTicksPerInch;
	public static final double kPathFollowingTolerance = 0.20;

	@Override
	public String toString() {
		return "kQuickStopAccumulatorDecreaseRate " + kQuickStopAccumulatorDecreaseRate + "kQuickStopAccumulatorDecreaseThreshold "
				+ kQuickStopAccumulatorDecreaseThreshold + "kNegativeInertiaScalar " + kNegativeInertiaScalar + "kAlpha " + kAlpha + "kDriveTicksPerInch "
				+ kDriveTicksPerInch + "kDriveInchesPerDegree" + kDriveInchesPerDegree + "kDriveSpeedUnitConversion " + kDriveSpeedUnitConversion;
	}
}