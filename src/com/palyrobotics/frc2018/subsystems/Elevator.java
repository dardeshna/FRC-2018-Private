package com.palyrobotics.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.Gains;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.util.TalonSRXOutput;

import javax.swing.text.html.Option;
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

	//Variables used to check if elevator is at the top or bottom position
	private boolean isAtTop = false;
	private boolean isAtBottom = true;

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

		//Checks calibration if not calibrated and not in custom/hold state (checks in manual, idle, calibrating)
		//Exception: in hold but bottomed out, so applying 0 power anyway
		if(mState != ElevatorState.CUSTOM_POSITIONING && (mState != ElevatorState.HOLD
				|| (mState == ElevatorState.HOLD && mRobotState.elevatorBottomHFX)) && !isCalibrated()) {
			checkCalibration();
		}

		handleState(commands);
		checkTopBottom(mRobotState);

		//Execute update loop based on the current state
		//Does not switch between states, only performs actions
		switch(mState) {
			//Actual calibration logic is not done in the state machine
			case CALIBRATING:
				mOutput.setPercentOutput(Constants.kCalibratePower);
				break;
			case HOLD:
				//If at the bottom, supply no power
				if(isAtBottom) {
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

				//If calibrated, run limiting code for top & bottom
				if(isCalibrated()) {
					double distInchesFromBottom = Math.abs((mRobotState.elevatorPosition - getElevatorBottomPosition().get())/Constants.kElevatorTicksPerInch);
					double distInchesFromTop = Math.abs((getElevatorTopPosition().get() - mRobotState.elevatorPosition)/Constants.kElevatorTicksPerInch);

					if(commands.disableElevatorScaling || mRobotState.hasElevatorStickyFaults) {
						mOutput.setPercentOutput(Constants.kElevatorUncalibratedManualPower * mRobotState.operatorStickInput.getY());
					} else {
						//close to bottom
						if(distInchesFromBottom < Constants.kElevatorBottomScalingMarginInches) {
							//Near bottom scales max allowed speed
							//Going down is negative
							mOutput.setPercentOutput(Math.max(-Constants.kElevatorBottomScalingConstant * distInchesFromBottom
									/ Constants.kElevatorBottomScalingMarginInches, mRobotState.operatorStickInput.getY()));
						} else if(distInchesFromTop < Constants.kElevatorTopScalingMarginInches) {
							//Near top scales max allowed speed
							//Going up is positive
							mOutput.setPercentOutput(Math.min(Constants.kElevatorTopScalingConstant * distInchesFromTop
									/ Constants.kElevatorTopScalingMarginInches + Constants.kElevatorHoldVoltage, mRobotState.operatorStickInput.getY() + Constants.kElevatorHoldVoltage));
						} else {
							//in middle
							mOutput.setPercentOutput(mRobotState.operatorStickInput.getY());
						}
					}
				} else {
					//if not calibrated, limit speed
					mOutput.setPercentOutput(Constants.kElevatorUncalibratedManualPower * mRobotState.operatorStickInput.getY());
				}

				// Add to setpoint based on joystick
//				mElevatorWantedPosition = Optional.of(
//						mElevatorWantedPosition.orElse(0.0) - robotState.operatorStickInput.getY() * Constants.kElevatorClosedLoopManualControlPositionSensitivity);
////				 Clamp position between bottom and top
//				mElevatorWantedPosition = Optional.of(Math.max(
//						kElevatorTopPosition.orElse(-Constants.kElevatorTopBottomDifferenceInches * Constants.kElevatorTicksPerInch),
//						Math.min(
//								kElevatorBottomPosition.orElse(0.0),
//								mElevatorWantedPosition.get())
//				));
//				mOutput.setPosition(mElevatorWantedPosition.get(), Gains.elevatorPosition);
				break;
			case CUSTOM_POSITIONING:
				//Control loop
				mOutput.setPosition(mElevatorWantedPosition.get(), Gains.elevatorPosition);
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
	 * <li>{@link ElevatorState#CALIBRATING}: Sets the state to calibrate. If already calibrated, ignores the request and holds instead.</li>
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
		if(mRobotState.hasElevatorStickyFaults && mRobotState.gamePeriod == RobotState.GamePeriod.AUTO) {
			kElevatorBottomPosition = Optional.empty();
			kElevatorTopPosition = Optional.empty();
			mElevatorWantedPosition = Optional.empty();
			mState = ElevatorState.IDLE;
		} else if(mRobotState.hasElevatorStickyFaults && mRobotState.gamePeriod == RobotState.GamePeriod.TELEOP) {
			kElevatorBottomPosition = Optional.empty();
			kElevatorTopPosition = Optional.empty();
			mElevatorWantedPosition = Optional.empty();
			mState = ElevatorState.MANUAL_POSITIONING;
		} else if(commands.wantedElevatorState == ElevatorState.CALIBRATING) {
			if(!isCalibrated()) {
				mState = ElevatorState.CALIBRATING;
			}  else {
				commands.wantedElevatorState = ElevatorState.HOLD;
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
			if(isCalibrated()) {
				//Set the setpoint
				//If the desired custom positioning setpoint is different than what currently
				//exists, replace it
				if(!mElevatorWantedPosition.equals(Optional.of(kElevatorBottomPosition.get() + commands.robotSetpoints.elevatorPositionSetpoint.get() * Constants.kElevatorTicksPerInch))) {
					mElevatorWantedPosition = Optional.of(kElevatorBottomPosition.get() + commands.robotSetpoints.elevatorPositionSetpoint.get() * Constants.kElevatorTicksPerInch);
				}
			} else {
				//Assume bottom position is the bottom
                if(!mElevatorWantedPosition.equals(Optional.of(commands.robotSetpoints.elevatorPositionSetpoint.get() * Constants.kElevatorTicksPerInch))) {
                    mElevatorWantedPosition = Optional.of(commands.robotSetpoints.elevatorPositionSetpoint.get() * Constants.kElevatorTicksPerInch);
                }
            }
			mState = ElevatorState.CUSTOM_POSITIONING;
		} else {
			//For idle/manual positioning, just set it
			mState = commands.wantedElevatorState;
		}

		//If custom positioning is finished, hold it
		if(onTarget()) {
			//Hold it next cycle
			commands.wantedElevatorState = ElevatorState.HOLD;
		}
	}

	public boolean movingUpwards() {
		//
		if((mOutput.getControlMode() == ControlMode.PercentOutput || mOutput.getControlMode() == ControlMode.Velocity) && mOutput.getSetpoint() > Constants.kElevatorHoldVoltage) {
			return true;
		} else if(mOutput.getControlMode() == ControlMode.MotionMagic || mOutput.getControlMode() == ControlMode.Position) {

			//Check calibration. If not calibrated, assume the worst and return true.
			if(isCalibrated()) {
				//If the desired setpoint is above the top position, assume it's moving up
				if(mOutput.getSetpoint() > getElevatorTopPosition().get()) {
					return true;
				}
			} else return true;
		} else if(mOutput.getControlMode() == ControlMode.Current) {
			if(mOutput.getSetpoint() == 0) {
				return false;
			}
			return true;
		}
		return false;
	}

	/**
	 * Checks whether or not the elevator has topped/bottomed out.
	 * Uses both HFX and encoders as redundant checks.
	 *
	 * @param state the robot state, used to obtain encoder values
	 */
	private void checkTopBottom(RobotState state) {
		if(state.elevatorTopHFX || (isCalibrated() && state.elevatorPosition > kElevatorTopPosition.get())) {
			isAtTop = true;
		} else {
			isAtTop = false;
		}
		if(state.elevatorBottomHFX || (isCalibrated() && state.elevatorPosition < kElevatorBottomPosition.get())) {
			isAtBottom = true;
		} else {
			isAtBottom = false;
		}
	}
	/**
	 * Calibrates the bottom or top position values depending on which HFX is triggered. If the other position is not already set, set that as well.
	 */
	private void checkCalibration() {

		if(!isCalibrated()) {
			if(mRobotState.elevatorBottomHFX) {
				kElevatorBottomPosition = Optional.of(mRobotState.elevatorPosition);
				if(!kElevatorTopPosition.isPresent()) {
					kElevatorTopPosition = Optional.of(mRobotState.elevatorPosition + Constants.kElevatorTopBottomDifferenceInches * Constants.kElevatorTicksPerInch);
				}
			} else if(mRobotState.elevatorTopHFX) {
				kElevatorTopPosition = Optional.of(mRobotState.elevatorPosition);
				if(!kElevatorBottomPosition.isPresent()) {
					kElevatorBottomPosition = Optional.of(mRobotState.elevatorPosition - Constants.kElevatorTopBottomDifferenceInches * Constants.kElevatorTicksPerInch);
				}
			}
		} else {
			if((kElevatorBottomPosition.get() - mRobotState.elevatorPosition) > Constants.kElevatorHFXAcceptableError ||
					(mRobotState.elevatorPosition - kElevatorTopPosition.get()) > Constants.kElevatorHFXAcceptableError) {
				kElevatorTopPosition = Optional.empty();
				kElevatorBottomPosition = Optional.empty();
			}
		}
	}

	public boolean isCalibrated() {
	    return (kElevatorTopPosition.isPresent() && kElevatorBottomPosition.isPresent());
    }

    @Override
	public void start() {

	}

	@Override
	public void stop() {
		mElevatorWantedPosition = Optional.empty();
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

	public boolean getIsAtTop() {
		return isAtTop;
	}

	public boolean getIsAtBottom() {
		return isAtBottom;
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
		if(mState != ElevatorState.CUSTOM_POSITIONING || mRobotState.hasElevatorStickyFaults) {
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
}