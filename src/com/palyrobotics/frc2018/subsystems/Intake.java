package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * @author Justin and Jason
 */
public class Intake extends Subsystem {
	public static Intake instance = new Intake();

	public static Intake getInstance() {
		return instance;
	}
	
	public static void resetInstance() { instance = new Intake(); }
	
	private TalonSRXOutput mTalonOutput = new TalonSRXOutput();
	private DoubleSolenoid.Value mOpenCloseOutput = DoubleSolenoid.Value.kReverse;
	private DoubleSolenoid.Value mUpDownOutput = DoubleSolenoid.Value.kForward;

	public enum WheelState {
		INTAKING, IDLE, EXPELLING
	}

	public enum UpDownState {
		UP, DOWN
	}

	public enum OpenCloseState {
		OPEN, CLOSED
	}

	private WheelState mWheelState = WheelState.IDLE;
	private UpDownState mUpDownState = UpDownState.UP;
	private OpenCloseState mOpenCloseState = OpenCloseState.CLOSED;

	protected Intake() {
		super("Intake");
	}

	@Override
	public void start() {
		mWheelState = WheelState.IDLE;
		mUpDownState = UpDownState.UP;
		mOpenCloseState = OpenCloseState.CLOSED;
	}

	@Override
	public void stop() {
		mWheelState = WheelState.IDLE;
		mUpDownState = UpDownState.UP;
		mOpenCloseState = OpenCloseState.CLOSED;
	}

	@Override
	public void update(Commands commands, RobotState robotState) {
		mWheelState = commands.wantedIntakingState;
		mUpDownState = commands.wantedIntakeUpDownState;
		mOpenCloseState = commands.wantedIntakeOpenCloseState;

		switch(mWheelState) {
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
				break;
			case DOWN:
				mUpDownOutput = DoubleSolenoid.Value.kForward;
				break;
		}

		switch(mOpenCloseState) {
			case OPEN:
				mOpenCloseOutput = DoubleSolenoid.Value.kReverse;
				break;
			case CLOSED:
				mOpenCloseOutput = DoubleSolenoid.Value.kForward;
				break;
		}
	}

	public WheelState getIntakeState() {
		return mWheelState;
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
		return "Intake State: " + mWheelState + "\nOutput Control Mode: " + mTalonOutput.getControlMode() + "\nTalon Output: " + mTalonOutput.getSetpoint()
				+ "\n" + "\nOpen Close Output: " + mOpenCloseOutput + "\n" + "\nUp Down Output: " + mUpDownOutput + "\n";
	}
}
