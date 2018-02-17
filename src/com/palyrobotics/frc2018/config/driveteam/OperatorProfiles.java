package com.palyrobotics.frc2018.config.driveteam;

import com.palyrobotics.frc2018.config.Constants;

public class OperatorProfiles {
	public static void configureConstants() {
		switch(Constants.kOperatorName) {
			case JACOB:
				Constants.kElevatorTopScalingMarginInches = 17.0;
				Constants.kElevatorBottomScalingMarginInches = 11.0;
				Constants.kElevatorTopScalingConstant = 0.55;
				Constants.kElevatorBottomScalingConstant = 0.31;
				break;
		}
	}
}
