package com.palyrobotics.frc2018.subsystems;

import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.util.TalonSRXOutput;

import java.util.Optional;

public class Elevator extends Subsystem {
	private static Elevator instance = new Elevator("Elevator");

	public static Elevator getInstance() {
		return instance;
	}

	public static void resetInstance() {
		instance = new Elevator("Elevator");
	}

	public enum ElevatorState {
		CALIBRATING, //Moving to the bottom to trigger HFX
		HOLD, //Keeping the elevator position fixed
		MANUAL_POSITIONING, //Moving the elevator with the joystick
		CUSTOM_POSITIONING, //Moving the elevator with a control loop
		IDLE //Not moving
	}

	//The variable used in the state machine
	private ElevatorState mState;

	//Values for the bottom/top positions to be calibrated
	private Optional<Double> kElevatorBottomPosition = Optional.empty();
	private Optional<Double> kElevatorTopPosition = Optional.empty();

	//Used for specifying where to hold/move to
	private Optional<Double> mElevatorWantedPosition = Optional.empty();

	//Used to store the robot state for use in methods other than update()
	private RobotState mRobotState;

	//The subsystem output
	private TalonSRXOutput mOutput = new TalonSRXOutput();

	/**
	 * Constructor for Elevator, defaults state to calibrating.
	 *
	 * @param name
	 *            the name of the elevator
	 */
	protected Elevator(String name) {
		super(name);
		mState = ElevatorState.CALIBRATING;
	}

	/**
	 * Calibration is checked and the variable for the state machine is set after processing the wanted elevator state. State machine used for movement and
	 * clearing {@link Elevator#mElevatorWantedPosition} only.
	 *
	 * @param commands
	 *            used to obtain wanted elevator state
	 * @param robotState
	 *            used to obtain joystick input and sensor readings
	 */
	@Override
	public void update(Commands commands, RobotState robotState) {

		//Update for use in handleState()
		mRobotState = robotState;

		checkCalibration();

		handleState(commands);

		//Execute update loop based on the current state
		//Does not switch between states, only performs actions
		switch(mState) {
			//Actual calibration logic is not done in the state machine
			case CALIBRATING:
				mOutput.setPercentOutput(Constants.kCalibratePower);
				break;
			case HOLD:
				//If at the bottom, supply no power
				if(mRobotState.elevatorBottomHFX) {
					mOutput.setPercentOutput(0.0);
				} else {
					//Control loop to hold position otherwise
					mOutput.setPosition(mElevatorWantedPosition.get(), Gains.elevatorHold);
				}
				break;
			case MANUAL_POSITIONING:
				//Clear any existing wanted positions
				if(mElevatorWantedPosition.isPresent()) {
					mElevatorWantedPosition = Optional.empty();
				}

				//Move with joystick input
				mOutput.setPercentOutput(mRobotState.operatorStickInput.getY());
				break;
			case CUSTOM_POSITIONING:
				//Control loop
				mOutput.setPosition(kElevatorBottomPosition.get() + mElevatorWantedPosition.get(), Gains.elevatorPosition);
				break;
			case IDLE:
				//Clear any existing wanted positions
				if(mElevatorWantedPosition.isPresent()) {
					mElevatorWantedPosition = Optional.empty();
				}

				mOutput.setPercentOutput(0.0);
				break;
			default:
				break;
		}
	}

	/**
	 * Process wanted elevator state and joystick inputs into mState for the state machine. Sets {@link Elevator#mElevatorWantedPosition} for use in the state
	 * machine. Does not clear it. At the end, always check if any custom positioning has finished, and if so, set the state to hold. <br>
	 * <br>
	 *
	 * <b>Teleop joystick movement overrides everything else!</b> <br>
	 * <br>
	 *
	 * Behavior for desired states:
	 * <ul>
	 * <li>{@link ElevatorState#CALIBRATING}: Sets the state to calibrate. If already calibrated, ignores the request and proceeds as usual.</li>
	 * <li>{@link ElevatorState#HOLD}: Sets the desired holding position and state to hold.</li>
	 * <li>{@link ElevatorState#MANUAL_POSITIONING}: Sets the state to manual.</li>
	 * <li>{@link ElevatorState#CUSTOM_POSITIONING}: Sets the desired custom position and state to custom positioning. If not calibrated, set to calibrate instead.</li>
	 * <li>{@link ElevatorState#IDLE}: Sets to idle.</li>
	 * </ul>
	 *
	 *
	 *
	 * @param commands
	 *            the commands used to get the wanted state
	 */
	private void handleState(Commands commands) {
		if(commands.wantedElevatorState == ElevatorState.CALIBRATING) {
			//If already calibrated, ignore it
			if(!kElevatorBottomPosition.isPresent() || !kElevatorTopPosition.isPresent()) {
				mState = ElevatorState.CALIBRATING;
			}
		} else if(commands.wantedElevatorState == ElevatorState.HOLD) {
			//Set the wanted elevator position if not already set, or if switching from a
			//different state
			if(!mElevatorWantedPosition.isPresent() || mState != commands.wantedElevatorState) {
				mElevatorWantedPosition = Optional.of(mRobotState.elevatorPosition);
			}
			mState = commands.wantedElevatorState;
		} else if(commands.wantedElevatorState == ElevatorState.CUSTOM_POSITIONING) {
			//If calibrated
			if(kElevatorBottomPosition.isPresent() && kElevatorTopPosition.isPresent()) {
				//Set the setpoint
				//If the desired custom positioning setpoint is different than what currently
				//exists, replace it
				if(!mElevatorWantedPosition.equals(Optional.of(kElevatorBottomPosition.get() + commands.robotSetpoints.elevatorPositionSetpoint.get()))) {
					mElevatorWantedPosition = Optional.of(kElevatorBottomPosition.get() + commands.robotSetpoints.elevatorPositionSetpoint.get());
				}

				mState = ElevatorState.CUSTOM_POSITIONING;

			} else {
				//If not calibrated, calibrate.
				mState = ElevatorState.CALIBRATING;
			}
		} else {
			//For idle/manual positioning, just set it
			mState = commands.wantedElevatorState;
		}

		//If custom positioning is finished, hold it
		if(onTarget()) {
			mState = ElevatorState.HOLD;
		}
	}

	/**
	 * Calibrates the bottom or top position values depending on which HFX is triggered. If the other position is not already set, set that as well.
	 */
	public void checkCalibration() {
		if(mRobotState.elevatorBottomHFX) {
			kElevatorBottomPosition = Optional.of(mRobotState.elevatorPosition);
			if(!kElevatorTopPosition.isPresent()) {
				kElevatorTopPosition = Optional.of(mRobotState.elevatorPosition + Constants.kTopBottomEncoderDifference);
			}
		} else if(mRobotState.elevatorTopHFX) {
			kElevatorTopPosition = Optional.of(mRobotState.elevatorPosition);
			if(!kElevatorBottomPosition.isPresent()) {
				kElevatorBottomPosition = Optional.of(mRobotState.elevatorPosition - Constants.kTopBottomEncoderDifference);
			}
		}
	}

	public void start() {

	}

	public void stop() {

	}

	public TalonSRXOutput getOutput() {
		return mOutput;
	}

	public Optional<Double> getElevatorBottomPosition() {
		return kElevatorBottomPosition;
	}

	public Optional<Double> getElevatorTopPosition() {
		return kElevatorTopPosition;
	}

	public Optional<Double> getElevatorWantedPosition() {
		return mElevatorWantedPosition;
	}

	/**
	 * If the elevator is on target. Only for {@link ElevatorState#CUSTOM_POSITIONING}.
	 *
	 * @return
	 *         <p>
	 *         false if {@link Elevator#mState} is not {@link ElevatorState#CUSTOM_POSITIONING}, or whether it's within position and velocity tolerances
	 *         otherwise
	 *         </p>
	 */
	public boolean onTarget() {
		if(mState != ElevatorState.CUSTOM_POSITIONING) {
			return false;
		}

		return (Math.abs(mElevatorWantedPosition.get() - mRobotState.elevatorPosition) < Constants.kElevatorAcceptablePositionError)
				&& (Math.abs(mRobotState.elevatorVelocity) < Constants.kElevatorAcceptableVelocityError);
	}

	public ElevatorState getState() {
		return mState;
	}

	//TODO: Find a better way to access these fields with unit tests only!
	public void setBottomPosition(Optional<Double> value) {
		kElevatorBottomPosition = value;
	}

	public void setTopPosition(Optional<Double> value) {
		kElevatorTopPosition = value;
	}
	
	public void setWantedPosition(double value) {
		mElevatorWantedPosition = Optional.of(value);
	}
}