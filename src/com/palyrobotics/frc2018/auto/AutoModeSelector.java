package com.palyrobotics.frc2018.auto;

import com.palyrobotics.frc2018.auto.AutoFMS.Side;
import com.palyrobotics.frc2018.auto.AutoModeBase.*;
import com.palyrobotics.frc2018.auto.modes.*;
import com.palyrobotics.frc2018.util.logger.Logger;
import org.json.simple.JSONArray;

import java.io.File;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.logging.Level;

/**
 * @author Nihar, based off Team 254 2015
 */
public class AutoModeSelector {
	private static AutoModeSelector instance = null;
	private ArrayList<AutoModeBase> mAutoModes = new ArrayList<>();
	private HashMap<String, Integer> mAutoMap = new HashMap<>();

	private AutoFMS autoFMS = AutoFMS.getInstance();

	public static AutoModeSelector getInstance() {
		if(instance == null) {
			instance = new AutoModeSelector();
		}
		return instance;
	}

	protected AutoModeSelector() {
		// left to right, blue alliance to red alliance
		/* 0 */ registerAutonomous(new BaselineAutoMode(Alliance.BLUE), 0);
		/* 1 */ registerAutonomous(new BaselineAutoMode(Alliance.RED), 1);

		/* 2 */ registerAutonomous(new LeftStartLeftSwitchAutoMode(Alliance.BLUE), 2);
		/* 3 */ registerAutonomous(new LeftStartLeftSwitchAutoMode(Alliance.RED), 3);

		/* 4 */ registerAutonomous(new CenterStartLeftSwitchAutoMode(Alliance.BLUE), 4);
		/* 5 */ registerAutonomous(new CenterStartLeftSwitchAutoMode(Alliance.RED), 5);

		/* 6 */ registerAutonomous(new RightStartLeftSwitchAutoMode(Alliance.BLUE), 6);
		/* 7 */ registerAutonomous(new RightStartLeftSwitchAutoMode(Alliance.RED), 7);

		/* 8 */ registerAutonomous(new LeftStartRightSwitchAutoMode(Alliance.BLUE), 8);
		/* 9 */ registerAutonomous(new LeftStartRightSwitchAutoMode(Alliance.RED), 9);

		/* 10 */ registerAutonomous(new CenterStartRightSwitchAutoMode(Alliance.BLUE), 10);
		/* 11 */ registerAutonomous(new CenterStartRightSwitchAutoMode(Alliance.RED), 11);

		/* 12 */ registerAutonomous(new RightStartRightSwitchAutoMode(Alliance.BLUE), 12);
		/* 13 */ registerAutonomous(new RightStartRightSwitchAutoMode(Alliance.RED), 13);

		/* 14 */ registerAutonomous(new LeftStartLeftScaleAutoMode(Alliance.BLUE), 14);
		/* 15 */ registerAutonomous(new LeftStartLeftScaleAutoMode(Alliance.RED), 15);

		/* 16 */ registerAutonomous(new CenterStartLeftScaleAutoMode(Alliance.BLUE), 16);
		/* 17 */ registerAutonomous(new CenterStartLeftScaleAutoMode(Alliance.RED), 17);

		/* 18 */ registerAutonomous(new RightStartLeftScaleAutoMode(Alliance.BLUE), 18);
		/* 19 */ registerAutonomous(new RightStartLeftScaleAutoMode(Alliance.RED), 19);

		/* 20 */ registerAutonomous(new LeftStartRightScaleAutoMode(Alliance.BLUE), 20);
		/* 21 */ registerAutonomous(new LeftStartRightScaleAutoMode(Alliance.RED), 21);

		/* 22 */ registerAutonomous(new CenterStartRightScaleAutoMode(Alliance.BLUE), 22);
		/* 23 */ registerAutonomous(new CenterStartRightScaleAutoMode(Alliance.RED), 23);

		/* 24 */ registerAutonomous(new RightStartRightScaleAutoMode(Alliance.BLUE), 24);
		/* 25 */ registerAutonomous(new RightStartRightScaleAutoMode(Alliance.RED), 25);

		/* 26 */ registerAutonomous(new CenterStartLeftMultiSwitchAutoMode(Alliance.BLUE), 26);
		/* 27 */ registerAutonomous(new CenterStartLeftMultiSwitchAutoMode(Alliance.RED), 27);

		/* 28 */ registerAutonomous(new CenterStartRightMultiSwitchAutoMode(Alliance.BLUE), 28);
		/* 29 */ registerAutonomous(new CenterStartRightMultiSwitchAutoMode(Alliance.RED), 29);

		/* 30 */ registerAutonomous(new LeftStartLeftMultiScaleAutoMode(Alliance.BLUE), 30);
		/* 31 */ registerAutonomous(new LeftStartLeftMultiScaleAutoMode(Alliance.RED), 31);

		/* 32 */ registerAutonomous(new LeftStartRightMultiScaleAutoMode(Alliance.BLUE), 32);
		/* 33 */ registerAutonomous(new LeftStartRightMultiScaleAutoMode(Alliance.RED), 33);

		/* 34 */ registerAutonomous(new RightStartLeftMultiScaleAutoMode(Alliance.BLUE), 34);
		/* 35 */ registerAutonomous(new RightStartLeftMultiScaleAutoMode(Alliance.RED), 35);

		/* 36 */ registerAutonomous(new RightStartRightMultiScaleAutoMode(Alliance.BLUE), 36);
		/* 37 */ registerAutonomous(new RightStartRightMultiScaleAutoMode(Alliance.RED), 37);
	}

	/**
	 * Add an AutoMode to list to choose from
	 *
	 * @param auto
	 *            AutoMode to add
	 */
	public void registerAutonomous(AutoModeBase auto, int id) {
		mAutoModes.add(auto);
		mAutoMap.put(auto.getKey(), id);
	}

	/**
	 * Get the currently selected AutoMode
	 *
	 * @return AutoMode currently selected
	 */
	public AutoModeBase getAutoMode() {
//		return new TestAutoMode();

//		return getAutoMode(AutoModeBase.mAlliance, AutoModeBase.mStartingPosition, AutoModeBase.mScaleDecision,
//                AutoModeBase.mSwitchDecision, AutoModeBase.mSecondScaleSideDecision, AutoModeBase.mSecondSwitchSideDecision,
//                AutoModeBase.mPriority, AutoModeBase.mSecondCubePriority, AutoModeBase.mMultiCube);

		return new RightStartRightScaleTwoCubeAutoMode(Alliance.BLUE);
	}

	/**
	 * Get the AutoMode at specified index
	 *
	 * @return AutoMode at specified index
	 */

	public AutoModeBase getAutoMode(Alliance alliance, StartingPosition startingPosition, Decision scaleDecision, Decision switchDecision, SecondSideDecision secondScaleDecision, SecondSideDecision secondSwitchDecision, Priority priority, Priority secondPriority, boolean multiCube) {
		//Final index
		int selectedIndex;

		//Index of the ideal scale and switch autos; the robot's priority determines which one is run
		int scaleIndex = -1;
		int switchIndex = -1;

		AutoFMS.Side scaleSide = autoFMS.getScaleSide();
		AutoFMS.Side switchSide = autoFMS.getSwitchSide();
		
		//Find ideal scale auto, or the auto it would execute if it had to score on the scale
		if(scaleDecision != Decision.NEVER) {
			if(scaleDecision == Decision.BOTH || (scaleDecision == Decision.LEFT && scaleSide == Side.LEFT) || scaleDecision == Decision.RIGHT && scaleSide == Side.RIGHT) {
				String key;
				if(multiCube && startingPosition != StartingPosition.CENTER) {
					key = alliance + " " + startingPosition + " SCALE " + scaleSide;
					String secondPartScale = "";
					String secondPartSwitch = "";
					if(secondScaleDecision != SecondSideDecision.NEVER) {
					    secondPartScale = " SCALE " + scaleSide;
                    }
                    if(secondSwitchDecision != SecondSideDecision.NEVER) {
					    if(secondSwitchDecision == SecondSideDecision.BOTH || (secondSwitchDecision == SecondSideDecision.SAME && scaleSide == switchSide) ||
                            (secondSwitchDecision == SecondSideDecision.OPPOSITE && scaleSide != switchSide)) {
                                secondPartSwitch = " SWITCH " + switchSide;
                        }
                    }
                    if(secondPriority == Priority.SWITCH) {
					    if(!secondPartSwitch.equals("")) {
					        key += secondPartSwitch;
                        } else if(!secondPartScale.equals("")) {
					        key += secondPartScale;
                        } else {
                            Logger.getInstance().logSubsystemThread(Level.WARNING, "Error in selecting auto, could not find a multi cube auto with the given specifications");
                        }
                    } else if(secondPriority == Priority.SCALE) {
					    if(!secondPartScale.equals("")) {
					        key += secondPartScale;
                        } else if(!secondPartSwitch.equals("")) {
					        key += secondPartSwitch;
                        } else {
                            Logger.getInstance().logSubsystemThread(Level.WARNING, "Error in selecting auto, could not find a multi cube auto with the given specifications");
                        }
                    }
				} else {
					key = alliance + " " + startingPosition + " SCALE " + scaleSide;
				}
                System.out.println("scale key: " + key);
				try {
					scaleIndex = mAutoMap.get(key);
				} catch (Exception e) {
					scaleIndex = 0;
					Logger.getInstance().logSubsystemThread(Level.WARNING, "Error in selecting auto, defaulting to baseline");
				}
				Logger.getInstance().logRobotThread(Level.INFO, "Attempted to find scale auto mode with this key: " + key);
			}
		}

		//Find ideal switch auto, or the auto it would execute if it had to score on the switch
		if(switchDecision != Decision.NEVER) {
            if (switchDecision == Decision.BOTH || (switchDecision == Decision.LEFT && switchSide == Side.LEFT) || switchDecision == Decision.RIGHT && switchSide == Side.RIGHT) {
                String key;
                if (multiCube && startingPosition != StartingPosition.CENTER) {
                    key = alliance + " " + startingPosition + " SWITCH " + switchSide;
                    String secondPartScale = "";
                    String secondPartSwitch = "";
                    if (secondScaleDecision != SecondSideDecision.NEVER) {
                        secondPartScale = " SCALE " + scaleSide;
                    }
                    if (secondSwitchDecision != SecondSideDecision.NEVER) {
                        if (secondSwitchDecision == SecondSideDecision.BOTH || (secondSwitchDecision == SecondSideDecision.SAME && scaleSide == switchSide) ||
                                (secondSwitchDecision == SecondSideDecision.OPPOSITE && scaleSide != switchSide)) {
                            secondPartSwitch = " SWITCH " + switchSide;
                        }
                    }
                    if (secondPriority == Priority.SWITCH) {
                        if (!secondPartSwitch.equals("")) {
                            key += secondPartSwitch;
                        } else if (!secondPartScale.equals("")) {
                            key += secondPartScale;
                        } else {
                            Logger.getInstance().logSubsystemThread(Level.WARNING, "Error in selecting auto, could not find a multi cube auto with the given specifications");
                        }
                    } else if (secondPriority == Priority.SCALE) {
                        if (!secondPartScale.equals("")) {
                            key += secondPartScale;
                        } else if (!secondPartSwitch.equals("")) {
                            key += secondPartSwitch;
                        } else {
                            Logger.getInstance().logSubsystemThread(Level.WARNING, "Error in selecting auto, could not find a multi cube auto with the given specifications");
                        }
                    } else {
                        key = alliance + " " + startingPosition + " SWITCH " + switchSide;
                    }
                    System.out.println("switch key: " + key);
                    try {
                        switchIndex = mAutoMap.get(key);
                    } catch (Exception e) {
                        switchIndex = 0;
                        Logger.getInstance().logSubsystemThread(Level.WARNING, "Error in selecting auto, defaulting to baseline");
                    }
                    Logger.getInstance().logRobotThread(Level.INFO, "Attempted to find switch auto mode with this key: " + key);
                }
            }
        }

		//Priority checking
		if(priority == Priority.SCALE) {
			if(scaleIndex != -1) {
				selectedIndex = scaleIndex;
			} else if(switchIndex != -1) {
				selectedIndex = switchIndex;
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "No auto mode possible, defaulting to baseline!");
				selectedIndex = mAutoMap.get(alliance.toString());
			}
		} else if(priority == Priority.SWITCH) {
			if(switchIndex != -1) {
				selectedIndex = switchIndex;
			} else if(scaleIndex != -1) {
				selectedIndex = scaleIndex;
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "No auto mode possible, defaulting to baseline!");
				selectedIndex = mAutoMap.get(alliance.toString());
			}
		} else {
			selectedIndex = (alliance == Alliance.BLUE) ? 0 : 1;
			Logger.getInstance().logRobotThread(Level.WARNING, "Priority not set, defaulting to baseline!");
		}

		AutoModeBase selectedAuto = mAutoModes.get(selectedIndex);
		Logger.getInstance().logRobotThread(Level.INFO, "Selected", selectedAuto);
		return selectedAuto;
	}

	/**
	 * Gets the names of all registered AutoModes
	 *
	 * @return ArrayList of AutoModes string name
	 * @see AutoModeBase#toString()
	 */
	public ArrayList<String> getAutoModeList() {
		ArrayList<String> list = new ArrayList<String>();
		for(AutoModeBase autoMode : mAutoModes) {
			list.add(autoMode.toString());
		}
		return list;
	}

	public JSONArray getAutoModeJSONList() {
		JSONArray list = new JSONArray();
		list.addAll(getAutoModeList());
		return list;
	}

	/**
	 * Attempt to set
	 *
	 * @return false if unable to find appropriate AutoMode
	 * @see AutoModeBase#toString()
	 */
	public AutoModeBase getAutoModeByName(String name) {
		if(!(mAutoMap.containsKey(name))) {
			Logger.getInstance().logRobotThread(Level.WARNING, "AutoModeSelector does not contain auto mode", name);
			return null;
		}
		int index = mAutoMap.get(name);
		Logger.getInstance().logRobotThread(Level.INFO, "Setting auto mode by name", name);
		return mAutoModes.get(index);
	}

	//TODO: what's this supposed to do
	/**
	 * Called during disabled in order to access dashboard and set auto mode
	 *
	 * @return
	 */
	public AutoModeBase getAutoModeFromDashboard(String selection) {
		return null;
	}

	public AutoModeBase getAutoModeByIndex(int index) {
		if(index < 0 || index >= mAutoModes.size()) {
			Logger.getInstance().logRobotThread(Level.WARNING, "Invalid AutoMode index, defautling to 0", index);
			index = 0;
		}
		Logger.getInstance().logRobotThread(Level.INFO, "Selected AutoMode by index", mAutoModes.get(index));
		return mAutoModes.get(index);
	}
}

