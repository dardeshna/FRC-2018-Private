package com.palyrobotics.frc2018.robot;

import com.palyrobotics.frc2018.auto.AutoFMS;
import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.auto.AutoModeSelector;
import com.palyrobotics.frc2018.behavior.RoutineManager;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.config.RobotState;
import com.palyrobotics.frc2018.config.dashboard.DashboardManager;
import com.palyrobotics.frc2018.config.driveteam.DriveTeam;
import com.palyrobotics.frc2018.config.fields.FieldSelector;
import com.palyrobotics.frc2018.subsystems.Climber;
import com.palyrobotics.frc2018.subsystems.Drive;
import com.palyrobotics.frc2018.subsystems.Elevator;
import com.palyrobotics.frc2018.subsystems.Intake;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;
import com.palyrobotics.frc2018.vision.VisionManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.logging.Level;

public class Robot extends TimedRobot {
	//Instantiate singleton classes
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
	private HardwareUpdater mHardwareUpdater = new HardwareUpdater(mDrive, mClimber, mElevator, mIntake);

	// Started boolean for if auto has been started.
	private boolean mAutoStarted = false;

	@Override
	public void robotInit() {
		Logger.getInstance().setFileName("2018 season");
		Logger.getInstance().start();

		Logger.getInstance().logRobotThread(Level.INFO, "Start robotInit() for " + Constants.kRobotName.toString());

		DashboardManager.getInstance().robotInit();
		//VisionManager.getInstance().start(Constants.kAndroidConnectionUpdateRate, false);

		Logger.getInstance().logRobotThread(Level.CONFIG, "Startup sucessful");
		Logger.getInstance().logRobotThread(Level.CONFIG, "Robot name: " + Constants.kRobotName);
		Logger.getInstance().logRobotThread(Level.CONFIG, "Alliance: " + DriverStation.getInstance().getAlliance());
		Logger.getInstance().logRobotThread(Level.CONFIG, "FMS connected: " + DriverStation.getInstance().isFMSAttached());
		Logger.getInstance().logRobotThread(Level.CONFIG, "Alliance station: " + DriverStation.getInstance().getLocation());
		try {
			Logger.getInstance().logRobotThread((VisionManager.getInstance().isServerStarted()) ? Level.CONFIG : Level.WARNING,
					(VisionManager.getInstance().isServerStarted()) ? "Nexus streaming" : "Nexus not streaming");
			Logger.getInstance().logRobotThread(Level.INFO, "Auto", AutoModeSelector.getInstance().getAutoMode().toString());
		} catch(NullPointerException e) {
			Logger.getInstance().logRobotThread(Level.SEVERE, "Auto", e);
		}

		mHardwareUpdater.initHardware();

		DriveTeam.configConstants();
		FieldSelector.configureFieldMeasurements();

		Logger.getInstance().logRobotThread(Level.INFO, "Auto" + AutoModeSelector.getInstance().getAutoMode().toString());
		Logger.getInstance().logRobotThread(Level.INFO, "End robotInit()");
	}

	@Override
	public void autonomousInit() {
		Logger.getInstance().start();
		Logger.getInstance().logRobotThread(Level.INFO, "Start autoInit()");

		DashboardManager.getInstance().toggleCANTable(true);
		robotState.gamePeriod = RobotState.GamePeriod.AUTO;
		mHardwareUpdater.configureHardware();

		//Wait for talons to update
		try {
			Logger.getInstance().logRobotThread(Level.FINEST, "Sleeping thread for 200 ms");
			Thread.sleep(200);
		} catch(InterruptedException e) {

		}

		mHardwareUpdater.updateState(robotState);
		mRoutineManager.reset(commands);
		robotState.reset(0, new RigidTransform2d());

		startSubsystems();

		Logger.getInstance().logRobotThread(Level.INFO, "End autoInit()");
	}


	@Override
	public void autonomousPeriodic() {
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

		logPeriodic();
	}

	@Override
	public void teleopInit() {
		Logger.getInstance().start();
		Logger.getInstance().logRobotThread(Level.INFO, "Start teleopInit()");

		robotState.gamePeriod = RobotState.GamePeriod.TELEOP;
		mHardwareUpdater.configureHardware();
		mHardwareUpdater.updateState(robotState);
		mHardwareUpdater.updateHardware();
		mRoutineManager.reset(commands);
		DashboardManager.getInstance().toggleCANTable(true);
		commands.wantedDriveState = Drive.DriveState.CHEZY; //switch to chezy after auto ends
		commands = operatorInterface.updateCommands(commands);
		startSubsystems();

		Logger.getInstance().logRobotThread(Level.INFO, "End teleopInit()");
	}

	@Override
	public void teleopPeriodic() {
		commands = mRoutineManager.update(operatorInterface.updateCommands(commands));
		mHardwareUpdater.updateState(robotState);
		updateSubsystems();

		//Update the hardware
		mHardwareUpdater.updateHardware();

		logPeriodic();
	}

	@Override
	public void disabledInit() {
		mAutoStarted = false;
		Logger.getInstance().start();

		robotState.gamePeriod = RobotState.GamePeriod.DISABLED;

		robotState.reset(0, new RigidTransform2d());

		//Stops updating routines
		mRoutineManager.reset(commands);

		//Creates a new Commands instance in place of the old one
		Commands.reset();
		commands = Commands.getInstance();

		//Stop controllers
		mDrive.setNeutral();
		mHardwareUpdater.configureHardware();
		mHardwareUpdater.disableTalons();
		DashboardManager.getInstance().toggleCANTable(false);

		stopSubsystems();

		//Manually run garbage collector
		System.gc();

		Logger.getInstance().logRobotThread(Level.INFO, "End disabledInit()");
	}

	@Override
	public void disabledPeriodic() {
	}

	//Call during teleop and auto periodic
	private void logPeriodic() {
		Logger.getInstance().logRobotThread(Level.FINEST, "Match time", DriverStation.getInstance().getMatchTime());
		Logger.getInstance().logRobotThread(Level.FINEST, "DS Connected", DriverStation.getInstance().isDSAttached());
		Logger.getInstance().logRobotThread(Level.FINEST, "DS Voltage", DriverStation.getInstance().getBatteryVoltage());
		Logger.getInstance().logRobotThread(Level.FINEST, "Outputs disabled", DriverStation.getInstance().isSysActive());
		Logger.getInstance().logRobotThread(Level.FINEST, "FMS connected", DriverStation.getInstance().isFMSAttached());
		if(DriverStation.getInstance().isAutonomous()) {
			Logger.getInstance().logRobotThread(Level.FINEST, "Game period: Auto");
		} else if(DriverStation.getInstance().isDisabled()) {
			Logger.getInstance().logRobotThread(Level.FINEST, "Game period: Disabled");
		} else if(DriverStation.getInstance().isOperatorControl()) {
			Logger.getInstance().logRobotThread(Level.FINEST, "Game period: Teleop");
		} else if(DriverStation.getInstance().isTest()) {
			Logger.getInstance().logRobotThread(Level.FINEST, "Game period: Test");
		}
		if(DriverStation.getInstance().isBrownedOut())
			Logger.getInstance().logRobotThread(Level.WARNING, "Browned out");
		if(!DriverStation.getInstance().isNewControlData())
			Logger.getInstance().logRobotThread(Level.FINE, "Didn't receive new control packet!");
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
}