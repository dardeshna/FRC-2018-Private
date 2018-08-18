package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.subsystems.Intake.OpenCloseState;
import com.palyrobotics.frc2018.subsystems.Intake.UpDownState;
import com.palyrobotics.frc2018.subsystems.Intake.WheelState;

public class IntakeSimulation extends SubsystemSimulation {
	
	private static IntakeSimulation instance = new IntakeSimulation();
	
	public static IntakeSimulation getInstance() {
		return instance;
	}
	
	WheelState wheel_state = WheelState.IDLE;
	OpenCloseState open_close_state = OpenCloseState.CLOSED;
	UpDownState up_down_state = UpDownState.UP;
	
	private int cycles_expelling;
	private int cycles_intaking;
	private boolean hasCube;
	
	public void step() {
		if (this.open_close_state == OpenCloseState.OPEN) {
			hasCube = false;
		}
		else {
			if (!hasCube && cycles_intaking > 300) {
				hasCube = true;
			}
			else if (hasCube && cycles_expelling > 100) {
				hasCube = false;
			}
		}
		
	}

	public void set(WheelState wheel_state, OpenCloseState open_close_state, UpDownState up_down_state) {
		
		if (this.wheel_state == wheel_state && wheel_state == WheelState.INTAKING) {
			cycles_intaking++;
		}
		else {
			cycles_intaking = 0;
		}
		
		if (this.wheel_state == wheel_state && wheel_state == WheelState.EXPELLING) {
			cycles_expelling++;
		}
		else {
			cycles_expelling = 0;
		}
		
		this.wheel_state = wheel_state;
		this.open_close_state = open_close_state;
		this.up_down_state = up_down_state;
	}


	public boolean hasCube() {
		return hasCube;
	}
}