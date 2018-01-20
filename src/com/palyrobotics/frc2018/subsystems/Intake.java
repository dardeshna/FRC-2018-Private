package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * @author Justin and Jason
 */
public class Intake extends Subsystem{
	public static Intake instance = new Intake();
	
	public static Intake getInstance() {
		return instance;
	}
	
	private TalonSRXOutput mTalonOutput = new TalonSRXOutput();
	private DoubleSolenoid.Value mOpenCloseOutput = DoubleSolenoid.Value.kReverse;
	private DoubleSolenoid.Value mUpDownOutput = DoubleSolenoid.Value.kForward;
	
	public enum IntakeState { 
		INTAKING,
		IDLE,
		EXPELLING
	}
	
	public enum UpDownState {
		UP,
		DOWN
	}
	
	public enum OpenCloseState {
		OPEN,
		CLOSED
	}
	
	private IntakeState mIntakeState = IntakeState.IDLE;
	private UpDownState mUpDownState = UpDownState.UP;
	private OpenCloseState mOpenCloseState = OpenCloseState.OPEN;
	
	public Intake() {
		super("Intake");
	}
	
	@Override
	public void start() {
		mIntakeState = IntakeState.IDLE;
		mUpDownState = UpDownState.UP;
		mOpenCloseState = OpenCloseState.OPEN;
	}
	
	@Override
	public void stop() {
		mIntakeState = IntakeState.IDLE;
		mUpDownState = UpDownState.UP;
		mOpenCloseState = OpenCloseState.OPEN;
	}
	
	@Override
	public void update(Commands commands, RobotState robotState) {
		mIntakeState = commands.wantedIntakeState;
		mUpDownState = commands.wantedUpDownState;
		mOpenCloseState = commands.wantedOpenCloseState;
		
		switch(mIntakeState) {
			case INTAKING:
				mTalonOutput.setPercentOutput(Constants.kIntakingMotorVelocity);
				break;
			case IDLE:
				mTalonOutput.setPercentOutput(0);
				break;
			case EXPELLING:
				mTalonOutput.setPercentOutput(Constants.kExpellingMotorVelocity);
				break;
		}
		
		switch(mUpDownState) {
			case UP:
				mUpDownOutput = DoubleSolenoid.Value.kReverse;
				mUpDownState = UpDownState.UP;
				break;
			case DOWN:
				mUpDownOutput = DoubleSolenoid.Value.kForward;
				mUpDownState = UpDownState.DOWN;
				break;
		}
		
		switch(mOpenCloseState) {
			case OPEN:
				mOpenCloseOutput = DoubleSolenoid.Value.kReverse;
				mOpenCloseState = OpenCloseState.OPEN;
				break;
			case CLOSED:
				mOpenCloseOutput = DoubleSolenoid.Value.kForward;
				mOpenCloseState = OpenCloseState.CLOSED;
				break;
		}
	}
	
	public IntakeState getIntakeState() {
		return mIntakeState;
	}
	
	public UpDownState getUpDownState() {
		return mUpDownState;
	}
	
	public OpenCloseState getOpenCloseState() {
		return mOpenCloseState;
	}
	
	public TalonSRXOutput getTalonOutput() {
		return mTalonOutput;
	}
	
	public DoubleSolenoid.Value getOpenCloseOutput() {
		return mOpenCloseOutput;
	}
	
	public DoubleSolenoid.Value getUpDownOutput() {
		return mUpDownOutput;
	}
	
	@Override
	public String getStatus() {
		return "Intake State: " + mIntakeState + "\nOutput Control Mode: " + mTalonOutput.getControlMode() + 
				"\nTalon Output: " + mTalonOutput.getSetpoint()+"\n" +
				"\nOpen Close Output: " + mOpenCloseOutput +"\n" +
				"\nUp Down Output: " + mUpDownOutput +"\n";
	}
}
