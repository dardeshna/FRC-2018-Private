package com.palyrobotics.frc2018.config.driveteam;

import com.palyrobotics.frc2018.config.Constants;

public class OperatorProfiles {
	public static void configureConstants() {
		switch(Constants.kOperatorName) {
			case JACOB:
				Constants.kElevatorTopScalingMarginInches = 18.0;
				Constants.kElevatorBottomScalingMarginInches = 12.0;
				Constants.kElevatorTopScalingConstant = 0.5;
				Constants.kElevatorBottomScalingConstant = 0.25;
				break;
		}
	}
}
