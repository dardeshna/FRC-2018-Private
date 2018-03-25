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
	    int index = 0;
        File folder = new File("src/com/palyrobotics/frc2018/auto/modes");
        for(File autoMode : folder.listFiles()) {
            String name = autoMode.getName().substring(0, autoMode.getName().length()-5);
            try {
                Class<?> autoClass = ClassLoader.getSystemClassLoader().loadClass("com.palyrobotics.frc2018.auto.modes." + name);
                Constructor<?> constructor = autoClass.getConstructor(Alliance.class);
                AutoModeBase blueAuto = (AutoModeBase) constructor.newInstance(Alliance.BLUE);
                registerAutonomous(blueAuto, index);
                index++;
                AutoModeBase redAuto = (AutoModeBase) constructor.newInstance(Alliance.RED);
                registerAutonomous(redAuto, index);
                index++;
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

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
//		return new FTestAuto();

		return getAutoMode(AutoModeBase.mAlliance, AutoModeBase.mStartingPosition, AutoModeBase.mScaleDecision,
                AutoModeBase.mSwitchDecision, AutoModeBase.mSecondScaleSideDecision, AutoModeBase.mSecondSwitchSideDecision,
                AutoModeBase.mPriority, AutoModeBase.mSecondCubePriority, AutoModeBase.mMultiCube);

//		return new RightStartRightScaleAutoMode(Alliance.BLUE);
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

