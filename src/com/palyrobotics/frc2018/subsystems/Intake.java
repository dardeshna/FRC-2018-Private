package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.robot.HardwareAdapter;
import com.palyrobotics.frc2018.util.TalonSRXOutput;
import com.palyrobotics.frc2018.util.csvlogger.CSVWriter;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * @author Justin and Jason and Prashanti
 */
public class Intake extends Subsystem {
	public static Intake instance = new Intake();

	public static Intake getInstance() {
		return instance;
	}
	
	public static void resetInstance() { instance = new Intake(); }
	
	private TalonSRXOutput mTalonOutput = new TalonSRXOutput();
	private boolean[] mOpenCloseOutput = new boolean[2];
	private DoubleSolenoid.Value mUpDownOutput = DoubleSolenoid.Value.kForward;

	public enum WheelState {
		INTAKING, IDLE, EXPELLING, VAULT_EXPELLING, AUTO_INTAKE
	}

	public enum UpDownState {
		UP, DOWN
	}

	public enum OpenCloseState {
		OPEN, CLOSED, NEUTRAL
	}

	private WheelState mWheelState = WheelState.IDLE;
	private UpDownState mUpDownState = UpDownState.UP;
	private OpenCloseState mOpenCloseState = OpenCloseState.CLOSED;

	private CSVWriter mWriter = CSVWriter.getInstance();


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
				if(commands.customIntakeSpeed) {
					mTalonOutput.setPercentOutput(robotState.operatorXboxControllerInput.leftTrigger);
				} else {
					mTalonOutput.setPercentOutput(Constants.kIntakingMotorVelocity);
				}
				break;
			case IDLE:
				mTalonOutput.setPercentOutput(0);
				break;
			case EXPELLING:
				if(commands.customIntakeSpeed) {
					mTalonOutput.setPercentOutput(-robotState.operatorXboxControllerInput.rightTrigger);
				} else {
					mTalonOutput.setPercentOutput(Constants.kExpellingMotorVelocity);
				}
				break;
			case VAULT_EXPELLING:
				mTalonOutput.setPercentOutput(Constants.kVaultExpellingMotorVelocity);
				break;
			case AUTO_INTAKE:
				mTalonOutput.setPercentOutput(Constants.kAutoIntakeVelocity);
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
				mOpenCloseOutput[0] = true;
				mOpenCloseOutput[1] = false;
				break;
			case CLOSED:
				mOpenCloseOutput[0] = false;
				mOpenCloseOutput[1] = true;
				break;
			case NEUTRAL:
				mOpenCloseOutput[0] = false;
				mOpenCloseOutput[1] = false;
				break;
		}

		mWriter.addData("intakeSetpoint", mTalonOutput.getSetpoint());
//		mWriter.addData("intakeCurrentDraw", HardwareAdapter.getInstance().getIntake().masterTalon.getOutputCurrent()
//				+ HardwareAdapter.getInstance().getIntake().slaveTalon.getOutputCurrent());
	}

	public WheelState getWheelState() {
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

	public boolean[] getOpenCloseOutput() {
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
