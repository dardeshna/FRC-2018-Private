package com.palyrobotics.frc2018.behavior.routines.intake;

import java.util.logging.Level;

import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.robot.Robot;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.subsystems.Subsystem;
import com.palyrobotics.frc2018.util.logger.Logger;

/**
 * @author Jason
 */
public class IntakeSensorStopRoutine extends Routine {
	
    private Intake.WheelState wantedWheelState;
    
    private RobotState mRobotState = Robot.getRobotState();
    
    //How long the wheels spin for (seconds)
  	private double mTimeout;
  	
  	private long mStartTime;

    public IntakeSensorStopRoutine(Intake.WheelState wantedWheelState, double timeout) {
        this.wantedWheelState = wantedWheelState;
        mTimeout = timeout;
    }

    @Override
    public void start() {
        mStartTime = System.currentTimeMillis();
    }

    @Override
    /**
     * Will only intake until cube is within robot, or expel until cube is fully out. 
     * Requires tuning to determine thresholds, relationship to max values.
     */
    public Commands update(Commands commands) {
        commands.wantedIntakingState = wantedWheelState;
        return commands;
    }

    @Override
    public Commands cancel(Commands commands) {
    	commands.wantedIntakingState = Intake.WheelState.IDLE;
        return commands;
    }

    @Override
    public boolean finished() {
    	if(wantedWheelState == Intake.WheelState.INTAKING && mRobotState.cubeDistance <= Constants.kIntakeCloseSensorThreshold) {
    		Logger.getInstance().logRobotThread(Level.INFO, "IntakeSensorStopRoutine finishing intake with cube at distance", mRobotState.cubeDistance);
    		return true;
    	} else if(wantedWheelState == Intake.WheelState.EXPELLING && mRobotState.cubeDistance >= Constants.kIntakeFarSensorThreshold) {
    		Logger.getInstance().logRobotThread(Level.INFO, "IntakeSensorStopRoutine finishing expel with cube at distance", mRobotState.cubeDistance);
    		return true;
    	} else if(System.currentTimeMillis() - mStartTime > mTimeout * 1000) {
    		Logger.getInstance().logRobotThread(Level.INFO, "IntakeSensorStopRoutine timeout", System.currentTimeMillis() - mStartTime);
    		return true;
    	} else {
            return false;
    	}
    }

    @Override
    public Subsystem[] getRequiredSubsystems() {
        return new Subsystem[] { intake };
    }
	@Override
	public String getName() {
        return "IntakeSensorStopRoutine";
	}
	
}