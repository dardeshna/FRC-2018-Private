package com.palyrobotics.frc2018.auto;

import java.util.ArrayList;
import java.util.logging.Level;

import com.palyrobotics.frc2018.auto.modes.*;
import org.json.simple.JSONArray;

import com.palyrobotics.frc2018.auto.AutoModeBase.Alliance;
import com.palyrobotics.frc2018.auto.AutoModeBase.Decision;
import com.palyrobotics.frc2018.auto.AutoModeBase.Priority;
import com.palyrobotics.frc2018.auto.AutoModeBase.StartingPosition;
import com.palyrobotics.frc2018.util.logger.Logger;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author Nihar, based off Team 254 2015
 */
public class AutoModeSelector {
	private static AutoModeSelector instance = null;
	private ArrayList<AutoModeBase> mAutoModes = new ArrayList<>();

	private AutoFMS autoFMS = AutoFMS.getInstance();

	public static AutoModeSelector getInstance() {
		if(instance == null) {
			instance = new AutoModeSelector();
		}
		return instance;
	}

	protected AutoModeSelector() {
		// left to right, blue alliance to red alliance
		/* 0 */ registerAutonomous(new BaselineAutoMode(Alliance.BLUE));
		/* 1 */ registerAutonomous(new BaselineAutoMode(Alliance.RED));

		/* 2 */ registerAutonomous(new LeftStartLeftSwitchAutoMode(Alliance.BLUE));
		/* 3 */ registerAutonomous(new LeftStartLeftSwitchAutoMode(Alliance.RED));

		/* 4 */ registerAutonomous(new CenterStartLeftSwitchAutoMode(Alliance.BLUE));
		/* 5 */ registerAutonomous(new CenterStartLeftSwitchAutoMode(Alliance.RED));

		/* 6 */ registerAutonomous(new RightStartLeftSwitchAutoMode(Alliance.BLUE));
		/* 7 */ registerAutonomous(new RightStartLeftSwitchAutoMode(Alliance.RED));

		/* 8 */ registerAutonomous(new LeftStartRightSwitchAutoMode(Alliance.BLUE));
		/* 9 */ registerAutonomous(new LeftStartRightSwitchAutoMode(Alliance.RED));

		/* 10 */ registerAutonomous(new CenterStartRightSwitchAutoMode(Alliance.BLUE));
		/* 11 */ registerAutonomous(new CenterStartRightSwitchAutoMode(Alliance.RED));

		/* 12 */ registerAutonomous(new RightStartRightSwitchAutoMode(Alliance.BLUE));
		/* 13 */ registerAutonomous(new RightStartRightSwitchAutoMode(Alliance.RED));

		/* 14 */ registerAutonomous(new LeftStartLeftScaleAutoMode(Alliance.BLUE));
		/* 15 */ registerAutonomous(new LeftStartLeftScaleAutoMode(Alliance.RED));

		/* 16 */ registerAutonomous(new CenterStartLeftScaleAutoMode(Alliance.BLUE));
		/* 17 */ registerAutonomous(new CenterStartLeftScaleAutoMode(Alliance.RED));

		/* 18 */ registerAutonomous(new RightStartLeftScaleAutoMode(Alliance.BLUE));
		/* 19 */ registerAutonomous(new RightStartLeftScaleAutoMode(Alliance.RED));

		/* 20 */ registerAutonomous(new LeftStartRightScaleAutoMode(Alliance.BLUE));
		/* 21 */ registerAutonomous(new LeftStartRightScaleAutoMode(Alliance.RED));

		/* 22 */ registerAutonomous(new CenterStartRightScaleAutoMode(Alliance.BLUE));
		/* 23 */ registerAutonomous(new CenterStartRightScaleAutoMode(Alliance.RED));

		/* 24 */ registerAutonomous(new RightStartRightScaleAutoMode(Alliance.BLUE));
		/* 25 */ registerAutonomous(new RightStartRightScaleAutoMode(Alliance.RED));

	}

	/**
	 * Add an AutoMode to list to choose from
	 *
	 * @param auto
	 *            AutoMode to add
	 */
	public void registerAutonomous(AutoModeBase auto) {
		mAutoModes.add(auto);
	}

	/**
	 * Get the currently selected AutoMode
	 * 
	 * @return AutoMode currently selected
	 */
	public AutoModeBase getAutoMode() {
//		return getAutoModeByIndex(0);
//		return getAutoModeByName("test");
//		return getAutoModeFromDashboard("test");
		boolean fmsConnected = false;
		try {
			fmsConnected = DriverStation.getInstance().isFMSAttached();
		} catch(UnsatisfiedLinkError e) {
			System.out.println("FMS is not attached");
		}
		if(fmsConnected) {
			return getAutoMode(AutoModeBase.mAlliance, AutoModeBase.mStartingPosition, AutoModeBase.mScaleDecision, 
					AutoModeBase.mSwitchDecision, AutoModeBase.mPriority);
		}
		else {
			return getAutoModeByIndex(0);
		}
	}

	/**
	 * Get the AutoMode at specified index
	 * 
	 * @return AutoMode at specified index
	 */
	public AutoModeBase getAutoMode(Alliance alliance, StartingPosition startingPosition, Decision scaleDecision, Decision switchDecision, Priority priority) {

		//Final index
		int selectedIndex;

		//Intermediate indices
		int scaleIndex = -1;
		int switchIndex = -1;

		/**
		 * Below is the nastiest block of logic you will ever see in your life
		 * Here is how it works
		 *
		 * 1. Check left/center/right starting cases
		 * 2. In each case, check the actual scale position. Match with the desired decision, then match with alliance
		 * 3. In each case, check the actual switch position. Match with the desired decision, then match with alliance
		 *
		 * After these, check the scale/switch priority and set the desired index
		 * Have fun
		 */

		if(startingPosition == StartingPosition.LEFT) {
			//Scale decision block
			if(AutoFMS.getInstance().getScaleSide() == AutoFMS.Side.LEFT) {
				if(scaleDecision == Decision.LEFT || scaleDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						scaleIndex = 14;
					} else if(alliance == Alliance.RED) {
						scaleIndex = 15;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else if(AutoFMS.getInstance().getScaleSide() == AutoFMS.Side.RIGHT){
				if(scaleDecision == Decision.RIGHT || scaleDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						scaleIndex = 20;
					} else if(alliance == Alliance.RED) {
						scaleIndex = 21;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "Defaulting scale to baseline");
			}

			//Switch decision block
			if(AutoFMS.getInstance().getSwitchSide() == AutoFMS.Side.LEFT) {
				if(switchDecision == Decision.LEFT || switchDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						switchIndex = 2;
					} else if(alliance == Alliance.RED) {
						switchIndex = 3;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else if(AutoFMS.getInstance().getSwitchSide() == AutoFMS.Side.RIGHT) {
				if(switchDecision == Decision.RIGHT || switchDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						switchIndex = 8;
					} else if(alliance == Alliance.RED) {
						switchIndex = 9;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "No side received from FMS, defaulting switch to baseline");
			}
		} else if(startingPosition == StartingPosition.CENTER) {
			//Scale decision block
			if(AutoFMS.getInstance().getScaleSide() == AutoFMS.Side.LEFT) {
				if(scaleDecision == Decision.LEFT || scaleDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						scaleIndex = 16;
					} else if(alliance == Alliance.RED) {
						scaleIndex = 17;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else if(AutoFMS.getInstance().getScaleSide() == AutoFMS.Side.RIGHT){
				if(scaleDecision == Decision.RIGHT || scaleDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						scaleIndex = 22;
					} else if(alliance == Alliance.RED) {
						scaleIndex = 23;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "Defaulting scale to baseline");
			}

			//Switch decision block
			if(AutoFMS.getInstance().getSwitchSide() == AutoFMS.Side.LEFT) {
				if(switchDecision == Decision.LEFT || switchDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						switchIndex = 4;
					} else if(alliance == Alliance.RED) {
						switchIndex = 5;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else if(AutoFMS.getInstance().getSwitchSide() == AutoFMS.Side.RIGHT) {
				if(switchDecision == Decision.RIGHT || switchDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						switchIndex = 10;
					} else if(alliance == Alliance.RED) {
						switchIndex = 11;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "Defaulting switch to baseline");
			}

		} else if(startingPosition == StartingPosition.RIGHT) {
			//Scale decision block
			if(AutoFMS.getInstance().getScaleSide() == AutoFMS.Side.LEFT) {
				if(scaleDecision == Decision.LEFT || scaleDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						scaleIndex = 18;
					} else if(alliance == Alliance.RED) {
						scaleIndex = 19;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else if(AutoFMS.getInstance().getScaleSide() == AutoFMS.Side.RIGHT){
				if(scaleDecision == Decision.RIGHT || scaleDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						scaleIndex = 24;
					} else if(alliance == Alliance.RED) {
						scaleIndex = 25;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "Defaulting scale to baseline");
			}

			//Switch decision block
			if(AutoFMS.getInstance().getSwitchSide() == AutoFMS.Side.LEFT) {
				if(switchDecision == Decision.LEFT || switchDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						switchIndex = 6;
					} else if(alliance == Alliance.RED) {
						switchIndex = 7;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else if(AutoFMS.getInstance().getSwitchSide() == AutoFMS.Side.RIGHT) {
				if(switchDecision == Decision.RIGHT || switchDecision == Decision.BOTH) {
					if(alliance == Alliance.BLUE) {
						switchIndex = 12;
					} else if(alliance == Alliance.RED) {
						switchIndex = 13;
					} else {
						Logger.getInstance().logRobotThread(Level.WARNING, "Alliance not set, defaulting to baseline");
					}
				}
			} else {
				Logger.getInstance().logRobotThread(Level.WARNING, "Defaulting switch to baseline");
			}
		} else {
			Logger.getInstance().logRobotThread(Level.WARNING, "Starting position not set, defaulting to baseline!");
		}

		//Priority checking
		if(priority == Priority.SCALE) {
			if(scaleIndex != -1) {
				selectedIndex = scaleIndex;
			} else if(switchIndex != -1) {
				selectedIndex = switchIndex;
			} else {
				selectedIndex = (alliance == Alliance.BLUE) ? 0 : 1;
			}
		} else if(priority == Priority.SWITCH) {
			if(switchIndex != -1) {
				selectedIndex = switchIndex;
			} else if(scaleIndex != -1) {
				selectedIndex = scaleIndex;
			} else {
				selectedIndex = (alliance == Alliance.BLUE) ? 0 : 1;
			}
		} else {
			selectedIndex = (alliance == Alliance.BLUE) ? 0 : 1;
			Logger.getInstance().logRobotThread(Level.WARNING, "Priority not set, defaulting to baseline!");
		}

		return mAutoModes.get(selectedIndex);
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
		int numOccurrences = 0;
		for(int i = 0; i < mAutoModes.size(); i++) {
			if(mAutoModes.get(i).toString().equals(name)) {
				return getAutoModeByIndex(i);
			}
		}

		Logger.getInstance().logRobotThread(Level.FINE, ((numOccurrences == 0) ? "Couldn't find AutoMode" : "Found multiple AutoModes"), name);
		Logger.getInstance().logRobotThread(Level.WARNING, "Didn't set AutoMode");
		return null;
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
			index = 0;
		}
		Logger.getInstance().logRobotThread(Level.INFO, "Selected AutoMode", mAutoModes.get(index));
		return mAutoModes.get(index);
	}

}
