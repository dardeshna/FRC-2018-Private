package com.palyrobotics.frc2018.behavior;

import com.palyrobotics.frc2018.behavior.routines.intake.*;
import com.palyrobotics.frc2018.config.Commands;
import com.palyrobotics.frc2018.config.MockCommands;
import com.palyrobotics.frc2018.config.MockRobotState;
import com.palyrobotics.frc2018.robot.MockRobot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.assertTrue;

import com.palyrobotics.frc2018.behavior.routines.intake.IntakeCloseRoutine;
import com.palyrobotics.frc2018.subsystems.Intake;

public class IntakeRoutineTest {
	
	Intake intake;
	
	MockRobotState robotState;
	
	MockCommands commands;
	
	long mStartTime;
	
	@Before
	public void init() {
		Intake.resetInstance();
		Commands.reset();
		
		robotState = MockRobot.getRobotState();
		intake = Intake.getInstance();
		commands = MockRobot.getCommands();
		
	}
	
	//Does each routine function properly when run independently?
	@Test
	public void testSingleRoutines() {

		Routine routine = new IntakeUpRoutine();
		routine.start();
		routine.update(commands);
		intake.update(commands, robotState);
		assertThat("Intake didn't flip up upon up routine", intake.getUpDownOutput(), equalTo(DoubleSolenoid.Value.kReverse));
		assertTrue("Up routine doesn't finish immediately after initiation", routine.finished());
		
		routine = new IntakeDownRoutine();
		routine.start();
		routine.update(commands);
		intake.update(commands, robotState);
		assertThat("Intake didn't flip down upon down routine", intake.getUpDownOutput(), equalTo(DoubleSolenoid.Value.kForward));
		assertTrue("Down routine doesn't finish immediately after initiation", routine.finished());
		
		routine = new IntakeCloseRoutine();
		routine.start();
		routine.update(commands);
		intake.update(commands, robotState);
		assertThat("Intake didn't close upon close routine", intake.getOpenCloseOutput(), equalTo(DoubleSolenoid.Value.kForward));
		assertTrue("Close routine doesn't finish immediately after initiation", routine.finished());
		
		routine = new IntakeOpenRoutine();
		routine.start();
		routine.update(commands);
		intake.update(commands, robotState);
		assertThat("Intake didn't open upon open routine", intake.getOpenCloseOutput(), equalTo(DoubleSolenoid.Value.kReverse));
		assertTrue("Open routine doesn't finish immediately after initiation", routine.finished());
		
		routine = new IntakeWheelRoutine(Intake.WheelState.INTAKING, 1);
		routine.start();
		mStartTime = System.currentTimeMillis();
		routine.update(commands);
		intake.update(commands, robotState);
		assertTrue("Intake didn't intake upon intaking routine", intake.getTalonOutput().getSetpoint() > 0);
		for (int i = 0; i < 1005; i++) {
			try {
				Thread.sleep(1);
			} catch(InterruptedException e) {
				e.printStackTrace();
			}
			routine.update(commands);
			assertFalse("Intaking routine timed out early", routine.finished() && System.currentTimeMillis() - mStartTime < 1000);
		}
		routine.update(commands);
		assertTrue("Intaking routine doesn't time out correctly", routine.finished());
		
		routine = new IntakeWheelRoutine(Intake.WheelState.EXPELLING, 1);
		routine.start();
		routine.update(commands);
		intake.update(commands, robotState);
		assertTrue("Intake didn't expel upon expel routine", intake.getTalonOutput().getSetpoint() < 0);
		for (int i = 0; i < 1005; i++) {
			try {
				Thread.sleep(1);
			} catch(InterruptedException e) {
				e.printStackTrace();
			}
			routine.update(commands);
			assertFalse("Expel routine timed out early", routine.finished() && System.currentTimeMillis() - mStartTime < 1000);
		}
		routine.update(commands);
		assertTrue("Expel routine doesn't finish immediately after initiation", routine.finished());
		
		routine = new IntakeWheelRoutine(Intake.WheelState.IDLE, 1);
		routine.start();
		routine.update(commands);
		intake.update(commands, robotState);
		assertTrue("Intake isn't idle upon idle routine", intake.getTalonOutput().getSetpoint() == 0);
		for (int i = 0; i < 1005; i++) {
			try {
				Thread.sleep(1);
			} catch(InterruptedException e) {
				e.printStackTrace();
			}
			routine.update(commands);
			assertFalse("Idle routine timed out early", routine.finished() && System.currentTimeMillis() - mStartTime < 1000);
		}
		routine.update(commands);
		assertTrue("Idle routine doesn't finish immediately after initiation", routine.finished());
	}
	
	@Test
	public void testComboRoutines() {
		RoutineManager routineManager = RoutineManager.getInstance();
		ArrayList<Routine> routines = new ArrayList<>();
		routines.add(new IntakeDownRoutine());
		routines.add(new IntakeOpenRoutine());
		routines.add(new IntakeWheelRoutine(Intake.WheelState.INTAKING, 1));
		routines.add(new IntakeCloseRoutine());
		routines.add(new IntakeWheelRoutine(Intake.WheelState.IDLE, 1));
		
		SequentialRoutine getCube = new SequentialRoutine(routines);
		routineManager.addNewRoutine(getCube);
		
		commands = (MockCommands) routineManager.update(commands);
		intake.update(commands, robotState);
		assertThat("Intake didn't flip down at the right point in the routine sequence", intake.getUpDownOutput(), equalTo(DoubleSolenoid.Value.kForward));
		
		commands = (MockCommands) routineManager.update(commands);
		intake.update(commands, robotState);
		assertThat("Intake didn't open at the right point in the routine sequence", intake.getOpenCloseOutput(), equalTo(DoubleSolenoid.Value.kReverse));
		
		commands = (MockCommands) routineManager.update(commands);
		mStartTime = System.currentTimeMillis();
		intake.update(commands, robotState);
		assertTrue("Intake didn't intake at the right point in the routine sequence", intake.getTalonOutput().getSetpoint() > 0);
		
		try {
			Thread.sleep(1000);
		} catch(InterruptedException e) {
			e.printStackTrace();
		}//while (System.currentTimeMillis() - mStartTime < 1000);
		
		commands = (MockCommands) routineManager.update(commands);
		intake.update(commands, robotState);
		
		commands = (MockCommands) routineManager.update(commands);
		intake.update(commands, robotState);
		assertThat("Intake didn't close at the right point in the routine sequence", intake.getOpenCloseOutput(), equalTo(DoubleSolenoid.Value.kForward));
		
		commands = (MockCommands) routineManager.update(commands);
		intake.update(commands, robotState);
		assertTrue("Intake isn't idle at the right point in the routine sequence", intake.getTalonOutput().getSetpoint() == 0);
	}
	
}
