package com.palyrobotics.frc2018.config.driveteam;

import com.palyrobotics.frc2018.config.Constants;

public class OperatorProfiles {
	public static void configureConstants() {
		switch(Constants.kOperatorName) {
			case JACOB:
				Constants.kElevatorTopScalingMarginInches = 11.0;
				Constants.kElevatorBottomScalingMarginInches = 18.0;
				Constants.kElevatorTopScalingConstant = 0.75;
				Constants.kElevatorBottomScalingConstant = 0.25;
				Constants.kElevatorUncalibratedManualPower = 0.7;
				Constants.kElevatorConstantDownPower = -0.2;
				Constants.operatorXBoxController = true;
				break;
		}
	}
}
