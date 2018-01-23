package com.palyrobotics.frc2017.suites;

import com.palyrobotics.frc2017.auto.AutoModeSelectorTest;
import com.palyrobotics.frc2017.behavior.RoutineManagerTest;
import com.palyrobotics.frc2017.config.CommandsTest;
import com.palyrobotics.frc2017.subsystems.DriveTest;
import com.palyrobotics.frc2017.util.CheesyDriveHelperTest;
import com.palyrobotics.frc2017.util.logger.LoggerTest;
import com.palyrobotics.frc2017.vision.AndroidComputerTest;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

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
 * Should not include integration tests, place in other suite
 */
public class UnitTestSuite {
  // the class remains empty,
  // used only as a holder for the above annotations
}