package com.palyrobotics.frc2018.robotplayback;

import java.time.LocalTime;
import java.util.ArrayList;

public class RecordedPath {
	
	public ArrayList<LocalTime> timestamps;
	public ArrayList<Double> xPositions, yPositions, headings, xLookaheads, yLookaheads;
	
	public String alliance;
	public String start_location;
	public int waypoints;
	public double[] waypointsX;
	public double[] waypointsY;
	
	public RecordedPath() {
		timestamps = new ArrayList<LocalTime>();
		xPositions = new ArrayList<Double>();
		yPositions = new ArrayList<Double>();
		headings = new ArrayList<Double>();
		xLookaheads = new ArrayList<Double>();
		yLookaheads = new ArrayList<Double>();
	}
	
	public RecordedPath(String alliance, String start_location, int waypoints, double[] waypointsX,
			double[] waypointsY) {
		this();
		this.alliance = alliance;
		this.start_location = start_location;
		this.waypoints = waypoints;
		this.waypointsX = waypointsX;
		this.waypointsY = waypointsY;
	}

	public void addPoint(LocalTime timestamp, double x, double y, double heading, double xLookahead, double yLookahead) {
		timestamps.add(timestamp);
		xPositions.add(x);
		yPositions.add(y);
		headings.add(heading);
		xLookaheads.add(xLookahead);
		yLookaheads.add(yLookahead);
	}
	
	public void print() {
		System.out.println("Path beginning at " + timestamps.get(0).toString() + "\n" + "------------------\n");
		System.out.println("Alliance: " + alliance);
		System.out.println("Location: " + start_location);
		System.out.println("Waypoints: " + waypoints);
		for (int i = 0; i < waypoints; i++) { 
			System.out.println("[" + waypointsX[i] + ", " + waypointsY[i]+"]");
		}
		System.out.println();
		for (int i = 0; i<timestamps.size(); i++) {
			System.out.println(timestamps.get(i) + ", " + xPositions.get(i) + ", " + yPositions.get(i));
		}
		System.out.println();
	}

}
