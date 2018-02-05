package com.palyrobotics.frc2018.vision.util.commandline;

import com.palyrobotics.frc2018.util.logger.Logger;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.logging.Level;

/**
 * Supplies wrapper methods for using ADB to control the Android
 *
 * <h1><b>Fields</b></h1>
 * <ul>
 * <li>Instance and State variables:
 * <ul>
 * <li>{@link RuntimeExecutor#s_Instance}: Private static instance of this class (Singleton)</li>
 * </ul>
 * </li>
 * </ul>
 *
 * <h1><b>Accessors and Mutators</b></h1>
 * <ul>
 * <li>{@link RuntimeExecutor#getInstance()}</li>
 * </ul>
 *
 * <h1><b>External Access Functions</b> <br>
 * <BLOCKQUOTE>For using as a wrapper for RIOdroid</BLOCKQUOTE></h1>
 * <ul>
 * <li>{@link RuntimeExecutor#exec(String)}</li>
 * </ul>
 *
 * @author Alvin
 *
 */
public class RuntimeExecutor {

	private static RuntimeExecutor s_Instance;

	private RuntimeExecutor() { }

	/**
	 * @return The singleton
	 */
	public static RuntimeExecutor getInstance() {
		if (s_Instance == null)
			s_Instance = new RuntimeExecutor();
		return s_Instance;
	}

	/**
	 * Executes a command in the command line during runtime
	 * 
	 * @param command Command to execute
	 * @return Console output of executing the command
	 */
	public String exec(final String command) {
		// Builds the output of the console
		final StringBuilder out = new StringBuilder();
		try {
			String line;
			// Execute the command as a process
			final Process p = Runtime.getRuntime().exec(command);
			// Read in console output from the process object
			final BufferedReader input = new BufferedReader(new InputStreamReader(p.getInputStream()));
			while ((line = input.readLine()) != null) {
				out.append(line);
				out.append("\n");
			}
			input.close();
		} catch (final IOException e) {
			Logger.getInstance().logRobotThread(Level.FINEST, e);
		}
		return out.toString();
	}
}
