package com.palyrobotics.frc2018.auto;

import java.util.ArrayList;
import java.util.Arrays;

import com.palyrobotics.frc2018.auto.AutoModeBase.Alliance;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.stage.Stage;

public class AutoPlayback extends Application {
	
	private static ArrayList<RigidTransform2d> poses = new ArrayList<>();
	
	private Group root;
	private Canvas canvas;
	
	// What point in the list of estimated positions are we displaying?
	private int currentPoseIndex = 0;
	
	private static final double canvasScale = 2;
	private static final double canvasWidth = 400 * canvasScale;
	private static final double canvasLength = 324 * canvasScale;

	public void start(Stage stage) throws Exception {
		
		stage.setTitle("Auto Playback");
		
		root = new Group();
		canvas = new Canvas(canvasWidth, canvasLength);
		root.getChildren().add(canvas);
		Scene scene = new Scene(root);
		stage.setScene(scene);
		
		new AnimationTimer() {

			@Override
			public void handle(long arg0) {
				GraphicsContext graphics = canvas.getGraphicsContext2D();
				graphics.setLineWidth(1.0);
				graphics.setStroke(new Color(0, 1, 0, 1));
				// Trace the path that's already been traveled
				for (int i = 0; i <= currentPoseIndex && i < poses.size(); i++) {
					RigidTransform2d pose = AutoPlayback.poses.get(i);
					double x = pose.getTranslation().getX(), y = pose.getTranslation().getY();
					// Draw one point in the path with radius 1
					graphics.fillOval(x - 1, y - 1, 2, 2);
				}
				// TODO: Draw the robot at its estimated position
				//Constants.kCenterOfRotationOffsetFromFrontInches
				currentPoseIndex++;
			}
			
		}.start();
		
		drawField(stage);
		
		stage.show();
	}
	
	// Receive the most recent estimated translation + rotation of the robot during auto.
	// This method should be called by AdaptivePurePursuit during every update cycle.
	public static void logPositionEstimation(RigidTransform2d robot_pose) {
		poses.add(robot_pose);
	}
	
	public static void main(String[] args) {
		launch(args);
	}
	
	// Called every update cycle: draws a basic Power Up field based on the deployed auto distances
	private void drawField(Stage stage) {
		GraphicsContext graphics = canvas.getGraphicsContext2D();
		
		// Since the positive y direction is down in JavaFX, y-coordinate calculations get nasty fast
		// TODO: Find a way to invert y-coordinates
		if (AutoModeBase.mAlliance == Alliance.RED) {
			graphics.setStroke(new Color(0, 0, 1, 1));
			graphics.setLineWidth(1.0);
			
			// Draw entire switch
			graphics.strokePolygon(converted(new double[] {AutoDistances.kBlueLeftSwitchX, AutoDistances.kBlueRightSwitchX, AutoDistances.kBlueRightSwitchX + AutoDistances.kSwitchPlateLength, AutoDistances.kBlueLeftSwitchX + AutoDistances.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kBlueLeftSwitchY, AutoDistances.kBlueRightSwitchY, AutoDistances.kBlueRightSwitchY, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftSwitchY}, false), 4);
			// Draw entire scale
			graphics.strokePolygon(converted(new double[] {AutoDistances.kBlueLeftScaleX, AutoDistances.kBlueRightScaleX, AutoDistances.kBlueRightScaleX + AutoDistances.kScalePlateLength, AutoDistances.kBlueLeftScaleX + AutoDistances.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kBlueLeftScaleY, AutoDistances.kBlueRightScaleY, AutoDistances.kBlueRightScaleY, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftScaleY}, false)  , 4);
			
			graphics.setLineWidth(3.0);
			
			// Draw left switch plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.kBlueLeftSwitchX, AutoDistances.kBlueLeftSwitchX, AutoDistances.kBlueLeftSwitchX + AutoDistances.kSwitchPlateLength, AutoDistances.kBlueLeftSwitchX + AutoDistances.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kBlueLeftSwitchY, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftSwitchY - AutoDistances.kSwitchPlateWidth, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftSwitchY - AutoDistances.kSwitchPlateWidth, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftSwitchY}, false), 4);
			// Draw right switch plate 
			graphics.strokePolygon(converted(new double[] {AutoDistances.kBlueRightSwitchX, AutoDistances.kBlueRightSwitchX, AutoDistances.kBlueRightSwitchX + AutoDistances.kSwitchPlateLength, AutoDistances.kBlueRightSwitchX + AutoDistances.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.kBlueRightSwitchY + AutoDistances.kSwitchPlateWidth, AutoDistances.kBlueRightSwitchY, AutoDistances.kBlueRightSwitchY, AutoDistances.kBlueRightSwitchY + AutoDistances.kSwitchPlateWidth}, false), 4);
		
			// Draw left scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.kBlueLeftScaleX, AutoDistances.kBlueLeftScaleX, AutoDistances.kBlueLeftScaleX + AutoDistances.kScalePlateLength, AutoDistances.kBlueLeftScaleX + AutoDistances.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kBlueLeftScaleY, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftScaleY - AutoDistances.kScalePlateWidth, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftScaleY - AutoDistances.kScalePlateWidth, AutoDistances.kFieldWidth - AutoDistances.kBlueLeftScaleY}, false), 4);
			// Draw right scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.kBlueRightScaleX, AutoDistances.kBlueRightScaleX, AutoDistances.kBlueRightScaleX + AutoDistances.kScalePlateLength, AutoDistances.kBlueRightScaleX + AutoDistances.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth, AutoDistances.kBlueRightScaleY, AutoDistances.kBlueRightScaleY, AutoDistances.kBlueRightScaleY + AutoDistances.kScalePlateWidth}, false), 4);
			
			// Draw pyramid square
			graphics.strokePolygon(converted(new double[] {AutoDistances.kBlueLeftSwitchX - AutoDistances.kBluePyramidLength, AutoDistances.kBlueLeftSwitchX - AutoDistances.kBluePyramidLength, AutoDistances.kBlueLeftSwitchX, AutoDistances.kBlueLeftSwitchX}, true),
									converted(new double[] {AutoDistances.kBluePyramidFromRightY + AutoDistances.kBluePyramidWidth, AutoDistances.kBluePyramidFromRightY, AutoDistances.kBluePyramidFromRightY, AutoDistances.kBluePyramidFromRightY + AutoDistances.kBluePyramidWidth}, false), 4);
		} else {
			graphics.setStroke(new Color(1, 0, 0, 1));
			graphics.setLineWidth(1.0);
			
			// Draw entire switch
			graphics.strokePolygon(converted(new double[] {AutoDistances.kRedLeftSwitchX, AutoDistances.kRedRightSwitchX, AutoDistances.kRedRightSwitchX + AutoDistances.kSwitchPlateLength, AutoDistances.kRedLeftSwitchX + AutoDistances.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kRedLeftSwitchY, AutoDistances.kRedRightSwitchY, AutoDistances.kRedRightSwitchY, AutoDistances.kFieldWidth - AutoDistances.kRedLeftSwitchY}, false), 4);
			// Draw entire scale
			graphics.strokePolygon(converted(new double[] {AutoDistances.kRedLeftScaleX, AutoDistances.kRedRightScaleX, AutoDistances.kRedRightScaleX + AutoDistances.kScalePlateLength, AutoDistances.kRedLeftScaleX + AutoDistances.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kRedLeftScaleY, AutoDistances.kRedRightScaleY, AutoDistances.kRedRightScaleY, AutoDistances.kFieldWidth - AutoDistances.kRedLeftScaleY}, false)  , 4);
			
			graphics.setLineWidth(3.0);
			
			// Draw left switch plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.kRedLeftSwitchX, AutoDistances.kRedLeftSwitchX, AutoDistances.kRedLeftSwitchX + AutoDistances.kSwitchPlateLength, AutoDistances.kRedLeftSwitchX + AutoDistances.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kRedLeftSwitchY, AutoDistances.kFieldWidth - AutoDistances.kRedLeftSwitchY - AutoDistances.kSwitchPlateWidth, AutoDistances.kFieldWidth - AutoDistances.kRedLeftSwitchY - AutoDistances.kSwitchPlateWidth, AutoDistances.kFieldWidth - AutoDistances.kRedLeftSwitchY}, false), 4);
			// Draw right switch plate 
			graphics.strokePolygon(converted(new double[] {AutoDistances.kRedRightSwitchX, AutoDistances.kRedRightSwitchX, AutoDistances.kRedRightSwitchX + AutoDistances.kSwitchPlateLength, AutoDistances.kRedRightSwitchX + AutoDistances.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.kRedRightSwitchY + AutoDistances.kSwitchPlateWidth, AutoDistances.kRedRightSwitchY, AutoDistances.kRedRightSwitchY, AutoDistances.kRedRightSwitchY + AutoDistances.kSwitchPlateWidth}, false), 4);
		
			// Draw left scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.kRedLeftScaleX, AutoDistances.kRedLeftScaleX, AutoDistances.kRedLeftScaleX + AutoDistances.kScalePlateLength, AutoDistances.kRedLeftScaleX + AutoDistances.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.kFieldWidth - AutoDistances.kRedLeftScaleY, AutoDistances.kFieldWidth - AutoDistances.kRedLeftScaleY - AutoDistances.kScalePlateWidth, AutoDistances.kFieldWidth - AutoDistances.kRedLeftScaleY - AutoDistances.kScalePlateWidth, AutoDistances.kFieldWidth - AutoDistances.kRedLeftScaleY}, false), 4);
			// Draw right scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.kRedRightScaleX, AutoDistances.kRedRightScaleX, AutoDistances.kRedRightScaleX + AutoDistances.kScalePlateLength, AutoDistances.kRedRightScaleX + AutoDistances.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth, AutoDistances.kRedRightScaleY, AutoDistances.kRedRightScaleY, AutoDistances.kRedRightScaleY + AutoDistances.kScalePlateWidth}, false), 4);
		
			// Draw pyramid square
						graphics.strokePolygon(converted(new double[] {AutoDistances.kRedLeftSwitchX - AutoDistances.kRedPyramidLength, AutoDistances.kRedLeftSwitchX - AutoDistances.kRedPyramidLength, AutoDistances.kRedLeftSwitchX, AutoDistances.kRedLeftSwitchX}, true),
												converted(new double[] {AutoDistances.kRedPyramidFromRightY + AutoDistances.kRedPyramidWidth, AutoDistances.kRedPyramidFromRightY, AutoDistances.kRedPyramidFromRightY, AutoDistances.kRedPyramidFromRightY + AutoDistances.kRedPyramidWidth}, false), 4);
		}
	}
	
	// Convert the field coordinates (inches) to the coordinates on the canvas
	private double[] converted(double[] coordinates, boolean areXCoordinates) {
		double[] retval = new double[coordinates.length];
		for (int i = 0; i < coordinates.length; i++) {
			if (areXCoordinates) {
				retval[i] = canvasScale * coordinates[i];
			} else {
				retval[i] = canvasLength - canvasScale * coordinates[i];
			}
		}
		return retval;
	}
	
}
