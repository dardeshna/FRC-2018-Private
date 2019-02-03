package com.palyrobotics.frc2018.robot;

import java.util.concurrent.TimeUnit;
import java.util.logging.Level;

import org.junit.Test;

import com.google.common.base.Stopwatch;
import com.palyrobotics.frc2018.auto.AutoFMS;
import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.auto.AutoModeSelector;
import com.palyrobotics.frc2018.behavior.RoutineManager;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
//import com.palyrobotics.frc2018.config.dashboard.DashboardManager;
import com.palyrobotics.frc2018.config.driveteam.DriveTeam;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.logger.DataLogger;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;


/**
 * Created by EricLiu on 11/12/17.
 */
public class MockRobot {
	//Instantiate singleton classes
	
	{
		Timer.SetImplementation(new Timer.StaticInterface() {
	        @Override
	        public double getFPGATimestamp() {
	          return System.currentTimeMillis() / 1000.0;
	        }

	        @Override
	        public double getMatchTime() {
	          return 0;
	        }

	        @Override
	        public void delay(double seconds) {
	          try {
	            Thread.sleep((long) (seconds * 1e3));
	          } catch (InterruptedException ex) {
	            Thread.currentThread().interrupt();
	            throw new RuntimeException("Thread was interrupted", ex);
	          }
	        }

	        @Override
	        public Timer.Interface newTimer() {
	          return new Timer.Interface() {
	            private final Stopwatch m_stopwatch = Stopwatch.createUnstarted();

	            @Override
	            public double get() {
	              return m_stopwatch.elapsed(TimeUnit.SECONDS);
	            }

	            @Override
	            public void reset() {
	              m_stopwatch.reset();
	            }

	            @Override
	            public void start() {
	              m_stopwatch.start();
	            }

	            @Override
	            public void stop() {
	              m_stopwatch.stop();
	            }

	            @Override
	            public boolean hasPeriodPassed(double period) {
	              if (get() > period) {
	                // Advance the start time by the period.
	                // Don't set it to the current time... we want to avoid drift.
	                m_stopwatch.reset().start();
	                return true;
	              }
	              return false;
	            }
	          };
	        }
		});
    }
		
	private static RobotState robotState = RobotState.getInstance();

	public static RobotState getRobotState() {
		return robotState;
	}

	//Single instance to be passed around
	private static Commands commands = Commands.getInstance();

	public static Commands getCommands() {
		return commands;
	}

	private OperatorInterface operatorInterface = OperatorInterface.getInstance();
	private RoutineManager mRoutineManager = RoutineManager.getInstance();

	//Subsystem controllers
	private Drive mDrive = Drive.getInstance();
	private Climber mClimber = Climber.getInstance();
	private Elevator mElevator = Elevator.getInstance();
	private Intake mIntake = Intake.getInstance();

	//Hardware Updater
	private HardwareUpdater mHardwareUpdater = new MockHardwareUpdater(mDrive, mClimber, mElevator, mIntake);

	// Started boolean for if auto has been started.
	private boolean mAutoStarted = false;

	
	public void robotInit() {
		Logger.getInstance().setFileName("AutoSim");
		Logger.getInstance().start();
		DataLogger.getInstance().setFileName("AutoSim");
		DataLogger.getInstance().start();

		Logger.getInstance().logRobotThread(Level.INFO, "Start robotInit() for " + Constants.kRobotName.toString());

//			DashboardManager.getInstance().robotInit();
//			VisionManager.getInstance().start(Constants.kVisionManagerUpdateRate, false);

		Logger.getInstance().logRobotThread(Level.CONFIG, "Startup successful");
		Logger.getInstance().logRobotThread(Level.CONFIG, "Robot name: " + Constants.kRobotName);
		mHardwareUpdater.initHardware();

		DriveTeam.configConstants();
		
		Logger.getInstance().logRobotThread(Level.INFO, "End robotInit()");
	}

	
	public void autonomousInit() {
		Logger.getInstance().start();
		Logger.getInstance().logRobotThread(Level.INFO, "Start autoInit()");

//			DashboardManager.getInstance().toggleCANTable(true);
		robotState.gamePeriod = RobotState.GamePeriod.AUTO;
		mHardwareUpdater.configureHardware();

		//Wait for talons to update
//			try {
//				Logger.getInstance().logRobotThread(Level.FINEST, "Sleeping thread for 200 ms");
//				Thread.sleep(200);
//			} catch(InterruptedException e) {
//
//			}

		mHardwareUpdater.updateState(robotState);
		mRoutineManager.reset(commands);
		robotState.reset(0, new RigidTransform2d());
//			commands.wantedIntakeUpDownState = Intake.UpDownState.UP;

		AutoDistances.updateAutoDistances();

		startSubsystems();
		mHardwareUpdater.enableBrakeMode();
		
		if(!AutoFMS.isFMSDataAvailable()) {
			Logger.getInstance().logRobotThread(Level.WARNING, "No FMS data detected");
		}

		Logger.getInstance().logRobotThread(Level.INFO, "End autoInit()");
	}


	
	public void autonomousPeriodic() {
		//if(!this.mAutoStarted) {
		if(AutoFMS.isFMSDataAvailable() && !this.mAutoStarted) {
			//Get the selected auto mode
			AutoModeBase mode = AutoModeSelector.getInstance().getAutoMode();

			//Prestart and run the auto mode
			mode.prestart();
			mRoutineManager.addNewRoutine(mode.getRoutine());

			this.mAutoStarted = true;
		}
		if(this.mAutoStarted) {
			commands = mRoutineManager.update(commands);
			mHardwareUpdater.updateState(robotState);
			updateSubsystems();
			mHardwareUpdater.updateHardware();
		}
//			System.out.println(mRoutineManager.getCurrentRoutines().contains(new DriveSensorResetRoutine(1.0)));
//			System.out.println("Position: " + Robot.getRobotState().getLatestFieldToVehicle().getValue());
		logPeriodic();
	}

	
	public void teleopInit() {
		Logger.getInstance().start();
		Logger.getInstance().logRobotThread(Level.INFO, "Start teleopInit()");
		commands.wantedIntakeUpDownState = Intake.UpDownState.DOWN;
		robotState.gamePeriod = RobotState.GamePeriod.TELEOP;
		robotState.reset(0.0, new RigidTransform2d());
		mHardwareUpdater.updateState(robotState);
		mHardwareUpdater.updateHardware();
		mRoutineManager.reset(commands);
//			DashboardManager.getInstance().toggleCANTable(true);
		commands.wantedDriveState = Drive.DriveState.CHEZY; //switch to chezy after auto ends
		commands = operatorInterface.updateCommands(commands);
//			commands.wantedIntakeUpDownState = Intake.UpDownState.DOWN;
		startSubsystems();
		mHardwareUpdater.enableBrakeMode();
		robotState.reset(0, new RigidTransform2d());
//			VisionManager.getInstance().verifyVisionAppIsRunning();

		Logger.getInstance().logRobotThread(Level.INFO, "End teleopInit()");
	}

	
	public void teleopPeriodic() {
		commands = mRoutineManager.update(operatorInterface.updateCommands(commands));
		mHardwareUpdater.updateState(robotState);
		updateSubsystems();

		//Update the hardware
		mHardwareUpdater.updateHardware();
		logPeriodic();
	}

	
	public void disabledInit() {
		mAutoStarted = false;
		Logger.getInstance().start();


		robotState.reset(0, new RigidTransform2d());

		//Stops updating routines
		mRoutineManager.reset(commands);

		//Creates a new Commands instance in place of the old one
		Commands.reset();
		commands = Commands.getInstance();
		if(robotState.gamePeriod == RobotState.GamePeriod.AUTO) {
			commands.wantedIntakeUpDownState = Intake.UpDownState.DOWN;
		} else {
			commands.wantedIntakeUpDownState = Intake.UpDownState.UP;
		}

		robotState.gamePeriod = RobotState.GamePeriod.DISABLED;

		//Stop controllers
		mDrive.setNeutral();
//			mHardwareUpdater.disableTalons();
		mHardwareUpdater.disableBrakeMode();
//			DashboardManager.getInstance().toggleCANTable(false);

		stopSubsystems();

		//Manually run garbage collector
		System.gc();

		Logger.getInstance().logRobotThread(Level.INFO, "End disabledInit()");
	}

	
	public void disabledPeriodic() {
	}

	//Call during teleop and auto periodic
	private void logPeriodic() {
		((MockHardwareUpdater) mHardwareUpdater).logSimulations();
		DataLogger.getInstance().cycle();
	}

	private void startSubsystems() {
		mDrive.start();
		mClimber.start();
		mElevator.start();
		mIntake.start();
	}

	private void updateSubsystems() {
		mDrive.update(commands, robotState);
		mClimber.update(commands, robotState);
		mElevator.update(commands, robotState);
		mIntake.update(commands, robotState);
	}

	private void stopSubsystems() {
		mDrive.stop();
		mClimber.stop();
		mElevator.stop();
		mIntake.stop();
	}
	
	@Test
	public void run() {
		robotInit();
		disabledInit();
		autonomousInit();
		int t = 0;
		boolean nowIsSet;
		long now;
		while (t < 25*1000) {
			now = System.nanoTime();
			nowIsSet = true;
			autonomousPeriodic();
			t+=(int)(Constants.kNormalLoopsDt*1000);
			int s = 0;
			while (s < (int)(Constants.kNormalLoopsDt*1000)) {
				if (!nowIsSet) now = System.nanoTime();
				nowIsSet = true;
				((MockHardwareUpdater) mHardwareUpdater).updateSimulations();
				while (System.nanoTime() < now + 1e6) {
					//wait
				}
				s++;
				nowIsSet = false;
			}
		}
		disabledInit();
		System.out.println("Writing to logs...");
		now = System.nanoTime();
		while (System.nanoTime() < now + (long)5.1e8) {
			//wait
		}
		System.out.println("Terminated");
	}
}
