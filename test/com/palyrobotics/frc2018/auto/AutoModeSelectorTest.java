package com.palyrobotics.frc2018.auto;

import com.palyrobotics.frc2018.auto.modes.*;
import com.palyrobotics.frc2018.config.Constants;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.Assert.assertThat;

/**
 * Created by Nihar on 1/22/17. Tests {@link AutoModeSelector}
 */
public class AutoModeSelectorTest {
	/*
	 * TODO: possible tests 
	 * Test that you can get an auto mode 
	 * Test that the setAutoMode by name works 
	 * Test that registerAutonomous works 
	 * Test that you can get the list of auto modes
	 */

	//TODO add new automodes and test those
	AutoModeSelector auto;
	@Before
	public void setUp() {
		auto = AutoModeSelector.getInstance();
	}

	@Test
	public void testGetAutoMode() throws IndexOutOfBoundsException {
		//Using automodes registered in constructor
		assertThat("Incorrect auto mode retrieved", auto.getAutoMode().getClass(), equalTo(new BaselineAutoMode(AutoModeBase.Alliance.RED).getClass()));

		assertThat("Created wrong AutoMode from index outside bounds", auto.getAutoModeByIndex(-1), equalTo(auto.getAutoModeByIndex(0)));
	}

	@Test
	public void testAutoPaths() {
		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Baseline", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.LEFT, AutoModeBase.Decision.RIGHT, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new BaselineAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Baseline", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.LEFT, AutoModeBase.Decision.NEVER, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new BaselineAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Scale", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new LeftStartLeftScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Left Switch", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new LeftStartLeftSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Left Scale despite Opposite Scale", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new LeftStartRightScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Switch despite Opposite Switch", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new LeftStartRightSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));
		
		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Right Scale", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new RightStartRightScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Right Switch", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.RIGHT, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new RightStartRightSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Right Scale despite Opposite Scale", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new RightStartLeftScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));
		
		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Right Switch despite Opposite Switch", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.RIGHT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new RightStartLeftSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Scale from Center", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SCALE).getClass(), equalTo(new CenterStartLeftScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Switch from Center", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new CenterStartLeftSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Right Scale from Center", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.RIGHT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SCALE).getClass(), equalTo(new CenterStartRightScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));
		
		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Right Switch from Center", auto.getAutoMode(AutoModeBase.Alliance.BLUE, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new CenterStartRightSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Scale", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new LeftStartLeftScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Left Switch", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new LeftStartLeftSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Left Scale despite Opposite Scale", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new LeftStartRightScaleAutoMode(AutoModeBase.Alliance.BLUE).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Switch despite Opposite Switch", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.LEFT,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new LeftStartRightSwitchAutoMode(AutoModeBase.Alliance.BLUE).getClass()));
		
		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Right Scale", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new RightStartRightScaleAutoMode(AutoModeBase.Alliance.RED).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Right Switch", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.RIGHT, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new RightStartRightSwitchAutoMode(AutoModeBase.Alliance.RED).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Right Scale despite Opposite Scale", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SCALE).getClass(), equalTo(new RightStartLeftScaleAutoMode(AutoModeBase.Alliance.RED).getClass()));
		
		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Right Switch despite Opposite Switch", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.RIGHT,  AutoModeBase.Decision.RIGHT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new RightStartLeftSwitchAutoMode(AutoModeBase.Alliance.RED).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Scale from Center", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.LEFT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SCALE).getClass(), equalTo(new CenterStartLeftScaleAutoMode(AutoModeBase.Alliance.RED).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Left Switch from Center", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.LEFT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new CenterStartLeftSwitchAutoMode(AutoModeBase.Alliance.RED).getClass()));

		AutoFMS.getInstance().setSwitch(AutoFMS.Side.LEFT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.RIGHT);
		assertThat("Incorrect Right Scale from Center", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.RIGHT, AutoModeBase.Decision.BOTH, AutoModeBase.Priority.SCALE).getClass(), equalTo(new CenterStartRightScaleAutoMode(AutoModeBase.Alliance.RED).getClass()));
		
		AutoFMS.getInstance().setSwitch(AutoFMS.Side.RIGHT);
		AutoFMS.getInstance().setScale(AutoFMS.Side.LEFT);
		assertThat("Incorrect Right Switch from Center", auto.getAutoMode(AutoModeBase.Alliance.RED, AutoModeBase.StartingPosition.CENTER,  AutoModeBase.Decision.BOTH, AutoModeBase.Decision.RIGHT, AutoModeBase.Priority.SWITCH).getClass(), equalTo(new CenterStartRightSwitchAutoMode(AutoModeBase.Alliance.RED).getClass()));

	}

	@Test
	public void testGetAutoModeList() {
		//TODO: Hard coded is sketchy
		//TODO: sometimes test is run individually, "this one is registered during testRegisterAutonomous()"
		int numberOfAutoModes = 29;
		auto.registerAutonomous(new TestTrajectoryAutoMode(), 26);
		auto.registerAutonomous(new TestTrajectoryAutoMode(), 27);
		ArrayList<String> test = auto.getAutoModeList();

		assertThat("Not all auto modes were retrieved", test.size(), equalTo(numberOfAutoModes));
//		assertThat("Auto modes are incorrect", test, equalTo(expectedAutoModeList));
	}

	@Test
	public void testSetAutoModeByName() {
		//Intentionally register two copies of the same auto mode class
		auto.registerAutonomous(new TestTrajectoryAutoMode(), 0);
		auto.registerAutonomous(new TestTrajectoryAutoMode(), 0);
		assertThat("Should not set auto mode when duplicates exist", auto.getAutoModeByName("TestTrajectoryAutoMode"), equalTo(null));
		assertThat("Found auto mode when none exists", auto.getAutoModeByName("1234"), equalTo(null));
	}

	/**
	 * Test that new autonomous modes are registered
	 */
	@Test
	public void testRegisterAutonomous() {
		int initSize = auto.getAutoModeList().size();
		ArrayList<String> autoNames = auto.getAutoModeList();
		AutoModeBase newAuto = new TestTrajectoryAutoMode();
		autoNames.add(newAuto.toString());
		auto.registerAutonomous(newAuto,0);

		//Index of the newest auto mode should be the original list length
		assertThat("AutoMode was registered incorrectly", auto.getAutoModeByIndex(initSize), equalTo(newAuto));
		assertThat("AutoMode was registered incorrectly", auto.getAutoModeList(), equalTo(autoNames));
	}

	@Test
	public void ticksPerDegreeCalculation() {
		System.out.println(Constants.kDriveInchesPerDegree * Constants.kDriveTicksPerInch);
	}
}
