package com.palyrobotics.frc2018.config;

public class Constants {
	public enum RobotName {
		FORSETI,
		HAL
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
	public static final String kPackageName = "com.frc8.team8vision.visionapp2018";
	public static final int kVisionManagerUpdateRate = 100; //Update rate in milliseconds
	public static final int kVisionVideoReceiverUpdateRate = 10;
	public static final int kVisionVideoServerUpdateRate = 10;
	public static final int kVisionDataPort = 8009;
	public static final int kVisionVideoReceiverSocketPort = 8008;
	public static final int kVisionVideoSocketPort = 5800;
	public static final boolean kVisionUseTimeout = true;
	public static final int kVisionMaxTimeoutWait = 5000;
	public static final long kServerReconnectWait = 1000;

	/**
	 * Cheesy Drive Constants Set by DriverProfiles
	 */
	//Deadband for joysticks
	public static double kDeadband;
	public static double kMaxAccelRate;
	//Threshold for quickturn sensitivity change
	public static double kQuickTurnSensitivityThreshold;
	//Sensitivities for how fast non-quickturn turning is
	public static double kDriveSensitivity;
	//Sensitivities for quickturn
	public static double kQuickTurnSensitivity;
	public static double kPreciseQuickTurnSensitivity;
	//The rate at which the QuickStopAccumulator will decrease
	public static double kQuickStopAccumulatorDecreaseRate;
	//The value at which the QuickStopAccumulator will stop decreasing
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
	public static final double kNominalUpwardsOutput = 0.1;
	public static final double kElevatorTopBottomDifferenceInches = 83.4192803716;
	public static final double kElevatorHFXAcceptableError = 0.01;
	public static final double kElevatorSwitchPositionInches = 40.0;
	public static final double kElevatorCubeInTransitPositionInches = 8.0;
	public static final double kElevatorBottomPositionInches = 0;
	public static final double kElevatorHoldVoltage = 0.11;

	public static double kElevatorTopScalingMarginInches;
	public static double kElevatorBottomScalingMarginInches;
	public static double kElevatorTopScalingConstant;
	public static double kElevatorBottomScalingConstant;
	public static double kElevatorUncalibratedManualPower;
	public static double kElevatorClosedLoopManualControlPositionSensitivity = 500;//250;
	public static double kElevatorConstantDownPower;

	/*
	 * Control loop constants for both robots
	 */
	public static final double kTurnInPlacePower = 0.17; //for bang bang
	public static final double kCalibratePower = -0.28;
	public static final double kDriveMaxClosedLoopOutput = 1.0;
	public static final double kElevatorMaxClosedLoopOutput = 0.666;

	/**
	 * Unit conversions for Talons
	 */
	public static final double kDriveTicksPerInch = 4096 / (6.25 * Math.PI);
	public static final double kElevatorTicksPerInch = 4096 / (3 * 1.432 * Math.PI);
	public static final double kDriveInchesPerDegree = 0.99 * 21.5 / 90;
	public static final double kDriveSpeedUnitConversion = 4096 / (6.25 * Math.PI * 10);

	/**
	 * Physical robot Constants
	 */
	public static final double kRobotWidthInches = 34.0;
	public static final double kRobotLengthInches = 39.0;
	public static final double kCenterOfRotationOffsetFromFrontInches = 13.0;
	public static final double kNullZoneAllowableBack = 4;

	/**
	 * Tolerances
	 */
	public static final double kAcceptableDrivePositionError = 15;
	public static final double kAcceptableDriveVelocityError = 5;
	public static final double kAcceptableShortDrivePositionError = 1;
	public static final double kAcceptableShortDriveVelocityError = 3;
	public static final double kAcceptableTurnAngleError = 5;
	public static final double kAcceptableGyroZeroError = 3;
	public static final double kAcceptableEncoderZeroError = 50;

	public static final double kElevatorAcceptablePositionError = 40;
	public static final double kElevatorAcceptableVelocityError = 0.01;

	//Intake
	public static final double kIntakingMotorVelocity = 0.75;
	public static final double kExpellingMotorVelocity = -0.225;
	public static final double kVaultExpellingMotorVelocity = -0.70;
	public static final double kExpelToScoreTime = 0.5;
	public static final double kIntakeMasterStallCurrent = 70;
	public static final double kIntakeMasterIdleCurrent = 0.125;
	public static final double kIntakeMasterExpelCurrent = 1.5;
	public static final double kIntakeMasterVaultExpelCurrent = 6.0;
	public static final double kIntakeMasterIntakingCurrent = 6;
	public static final double kIntakeSlaveStallCurrent = 65;
	public static final double kIntakeSlaveIdleCurrent = 0.125;
	public static final double kIntakeSlaveExpelCurrent = 1.5;
	public static final double kIntakeSlaveVaultExpelCurrent = 6.0;
	public static final double kIntakeSlaveIntakingCurrent = 6;
	public static final double kSwitchAutoWaitBeforeElevatorRaiseTimeSeconds = 1.5;
	public static final double kScaleAutoWaitBeforeElevatorRaiseTimeSeconds = 1.0;
	public static final double kScaleOffset = 18;
	public static final int kIntakeStallCounterThreshold = 5;
	public static final int kIntakeFreeCounterThreshold = 1;
	public static final double kIntakeStallCoefficientA = 1.55;
	public static final double kIntakeStallCoefficientB = 4.27;
	public static final double kIntakeFreeSpinCurrent = 5;
	public static final double kIntakeCubeInchTolerance = 10.5;

	public static boolean operatorXBoxController = true;

	//Weird units, it's in sum of joystick inputs. 50 updates/sec, add 1.0 at max power every cycle, so this is half a second of max power
	public static final double kClimberUpPositionEstimateThreshold = 25;

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

	//Compressor
    public static final int kForsetiCompressorSensorDeviceID = 0;
    public static final int kForsetiCompressorDeviceID = 0;
    public static final int kForsetiCompressorVoltageToPSI = 0;
    public static final int kForsetiPressureCurrentProductThreshold = 0;
    public static final int kForsetiCompressorMaxPSI = 120;

	//DRIVETRAIN
	//PDP slots for drivetrain 0, 1, 2, 3, 12, 13
	public static final int kForsetiLeftDriveMasterDeviceID = 1;
	public static final int kForsetiLeftDriveSlave1DeviceID = 2;
	public static final int kForsetiLeftDriveSlave2DeviceID = 3;
	public static final int kForsetiLeftDriveSlave3DeviceID = 13;
	public static final int kForsetiLeftDriveFrontMotorPDP = 0;
	public static final int kForsetiLeftDriveBackMotorPDP = 0;
	public static final int kForsetiLeftDriveThirdMotorPDP = 0;
	public static final int kForsetiRightDriveMasterDeviceID = 4;
	public static final int kForsetiRightDriveSlave1DeviceID = 5;
	public static final int kForsetiRightDriveSlave2DeviceID = 6;
    public static final int kForsetiRightDriveSlave3DeviceID = 14;
    public static final int kForsetiRightDriveFrontMotorPDP = 0;
	public static final int kForsetiRightDriveBackMotorPDP = 0;
	public static final int kForsetiRightDriveThirdMotorPDP = 0;

	//CLIMBER
	public static final int kForsetiClimberMotorDeviceID = 8;

	//PCM 1 for now
	public static final int kForsetiClimberLatchSolenoidForwardID = 6;
	public static final int kForsetiClimberLatchSolenoidReverseID = 1;

	public static final int kForsetiClimberBrakeSolenoidID = 7;
	public static final int kHALClimberBrakeSolenoidID = 6;


	//ELEVATOR
	//PDP slots for elevator TBD (currently 7, 8, 9, 10)
	public static final int kForsetiElevatorMasterTalonID = 12;
	public static final int kForsetiElevatorSlaveTalonID = 11;

	//INTAKE
	public static final int kForsetiIntakeMasterDeviceID = 9;
	public static final int kForsetiIntakeSlaveDeviceID = 10;
	//PCM 0
	public static final int kForsetiIntakeUpDownSolenoidForwardID = 2;
	public static final int kForsetiIntakeUpDownSolenoidReverseID = 5;
	//PCM 1
	public static final int kForsetiIntakeOpenCloseOtherSolenoidID = 4;
	public static final int kForsetiIntakeOpenCloseSolenoidID = 6;
	public static final int kHALIntakeOpenCloseSolenoid = 5;
	//Ultrasonic
	public static final int kForsetiIntakeUltrasonicPing = 9;
	public static final int kForsetiIntakeUltrasonicEcho = 8;


	//!!! Physical constants
	public static final double kSquareCubeLength = 13.0;

	//!!! Loop rate of normal Looper
	public static final double kNormalLoopsDt = 0.02;

	//Adaptive Pure Pursuit Controller
	public static final double kDriveWheelDiameterInches = 6.25;
	public static final double kTrackLengthInches = 8.5;
	public static final double kTrackWidthInches = 24.0;
	public static final double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
	public static final double kTrackScrubFactor = 0.5;
	public static final double kPathFollowingLookahead = 35.0;
	public static final double kPathFollowingMaxAccel = 90;
	public static final double kPathFollowingMaxVel = 200.0;
	//public static final double kPathFollowingMaxVel = 5 * kPathFollowingMaxAccel;
	public static final double kPathFollowingTolerance = 0.20;

	@Override
	public String toString() {
		return "kQuickStopAccumulatorDecreaseRate " + kQuickStopAccumulatorDecreaseRate + "kQuickStopAccumulatorDecreaseThreshold "
				+ kQuickStopAccumulatorDecreaseThreshold + "kNegativeInertiaScalar " + kNegativeInertiaScalar + "kAlpha " + kAlpha + "kDriveTicksPerInch "
				+ kDriveTicksPerInch + "kDriveInchesPerDegree" + kDriveInchesPerDegree + "kDriveSpeedUnitConversion " + kDriveSpeedUnitConversion;
	}
}