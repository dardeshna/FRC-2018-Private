package com.palyrobotics.frc2017.suites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import com.palyrobotics.frc2017.util.*;
import com.palyrobotics.frc2017.util.logger.*;
import com.palyrobotics.frc2017.vision.AndroidComputerTest;
import com.palyrobotics.frc2017.subsystems.*;
import com.palyrobotics.frc2017.config.*;
import com.palyrobotics.frc2017.auto.*;
import com.palyrobotics.frc2017.behavior.*;
import com.palyrobotics.frc2017.robot.*;

@RunWith(Suite.class)
@Suite.SuiteClasses({
	DriveTest.class,
	CommandsTest.class,
	CheesyDriveHelperTest.class,
	LoggerTest.class,
	RoutineManagerTest.class,
	AutoModeSelectorTest.class,
	AndroidComputerTest.class
})
/**
 * Test suite for unit tests
 * @author Joseph Rumelhart
 *
 * All of the included tests should pass to verify integrity of code
 * Update with new unit tests as classes are created
 * Should not include integration or simulation tests, place in other suites
 */
public class UnitTestSuite {
  // the class remains empty,
  // used only as a holder for the above annotations
}