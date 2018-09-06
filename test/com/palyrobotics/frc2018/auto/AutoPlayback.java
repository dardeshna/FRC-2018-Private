package com.palyrobotics.frc2018.auto;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import com.palyrobotics.frc2018.auto.AutoModeBase.Alliance;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.PathSegment;
import com.palyrobotics.frc2018.util.trajectory.RigidTransform2d;
import com.palyrobotics.frc2018.util.trajectory.Rotation2d;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.shape.Polyline;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.Window;

public class AutoPlayback extends Application {
	
	// The sequence of estimated poses split into individual components
	private ArrayList<Double> xPositions, yPositions, headings, xLookaheads, yLookaheads;
	private Path mPath;
	
	private Group root;
	private Scene scene;
	private Canvas canvas;
	
	// Has playback been paused or not?
	private boolean paused = false;
	
	// What point in the list of estimated positions are we displaying?
	private int currentPoseIndex = 0;
	
	// Starting position of the robot as determined by AutoModeBase
	private static double startX = converted(Constants.kRobotLengthInches - Constants.kCenterOfRotationOffsetFromFrontInches, true);
	private static double startY = 0;
	
	private static final double canvasScale = 2;
	private static final double canvasWidth = 400 * canvasScale;
	private static final double canvasLength = 324 * canvasScale;

	public void start(Stage stage) throws Exception {
		
		resetPoses();
		handleInput(stage);
		
		stage.setTitle("Auto Playback");
		
		root = new Group();
		canvas = new Canvas(canvasWidth, canvasLength);
		root.getChildren().add(canvas);
		scene = new Scene(root);
		stage.setScene(scene);
		
		new AnimationTimer() {

			@Override
			public void handle(long arg0) {
				GraphicsContext graphics = canvas.getGraphicsContext2D();
				
				if (currentPoseIndex < xPositions.size()) {
					// Trace the path that's already been traveled
					// This isn't implemented with graphics.fillPolyline() because that would require repeated casting of 
					// the lists of coordinates
					graphics.setLineWidth(3.0);
					double x = startX + xPositions.get(currentPoseIndex) * canvasScale, y = startY - yPositions.get(currentPoseIndex) * canvasScale;
					double xLookahead = startX + xLookaheads.get(currentPoseIndex) * canvasScale, yLookahead = startY - yLookaheads.get(currentPoseIndex) * canvasScale;double rotationDegrees = -headings.get(currentPoseIndex);
					
					graphics.setStroke(new Color(0, 1, 0, 1));
					graphics.fillOval(x - graphics.getLineWidth() / 2.0, y - graphics.getLineWidth() / 2.0, graphics.getLineWidth(), graphics.getLineWidth());
				
					// Draw the current lookahead point
					Circle lookaheadPoint = new Circle(xLookahead, yLookahead, 5.0, new Color(0, 1, 1, 1));
					
					// Draw a line connecting the robot center of rotation and lookahead for reference
					Line lookaheadLine = new Line(x, y, xLookahead, yLookahead);
					lookaheadLine.setStroke(new Color(0, 1, 1, 1));
					lookaheadLine.setStrokeWidth(3.0);
					
					//Draw the robot at its estimated position
					Rectangle robot = new Rectangle(-Constants.kRobotLengthInches + Constants.kCenterOfRotationOffsetFromFrontInches, -Constants.kRobotWidthInches / 2.0, Constants.kRobotLengthInches, Constants.kRobotWidthInches);
					Scale scale = new Scale(canvasScale, canvasScale, 0, 0);
					Rotate rotation = new Rotate(rotationDegrees, 0, 0);
					Translate translation = new Translate(x, y);
					
					// NOTE: The transformations are applied in the reverse of the order you add them in!
					robot.getTransforms().addAll(translation, rotation, scale);
					robot.setStroke(new Color(0, 1, 0, 1));
					robot.setFill(new Color(1, 1, 1, 0));
					root = new Group();
					root.getChildren().add(canvas);
					root.getChildren().add(robot);
					root.getChildren().add(lookaheadLine);
					root.getChildren().add(lookaheadPoint);
					
					scene.setRoot(root);
					//stage.setScene(scene);

					if (!paused) currentPoseIndex++;
				} else paused = true;
				
			}
			
		}.start();
		
		// Configure keyboard control to stop, start, increment, and rewind playback
		scene.setOnKeyPressed(new EventHandler<KeyEvent>() {

			@Override
			public void handle(KeyEvent event) {
				switch (event.getCode()) {
				
				case SPACE:
					paused = !paused;
					break;
				case LEFT:
					if (paused) currentPoseIndex = Math.max(currentPoseIndex - 4, 0);
					break;
				case RIGHT:
					if (paused) currentPoseIndex = Math.min(currentPoseIndex + 4, xPositions.size() - 1);;
					break;
				case HOME:
					currentPoseIndex = 0;
				default:
					break;
				
				}
			}
			
		});
		
		drawField();
		drawPath();
		
		stage.show();
	}
	
	// Reset the list of received robot poses. This should be called at the start of auto
	public void resetPoses() {
		xPositions = new ArrayList<>();
		yPositions = new ArrayList<>();
		headings = new ArrayList<>();
		xLookaheads = new ArrayList<>();
		yLookaheads = new ArrayList<>();
	}
	
	public void setPath(Path path) {
		mPath = path;
	}
	
	public static void main(String[] args) {
		launch(args);
	}
	
	// Parse the auto log file selected by the user
	private void handleInput(Stage stage) {
		FileChooser fileChooser = new FileChooser();
		fileChooser.getExtensionFilters().add(new ExtensionFilter("Auto Log Files", "*.auto"));
		fileChooser.setTitle("Select an auto log file to visualize");
		File logFile = fileChooser.showOpenDialog(stage);
		
		Scanner scanner;
		try {
			scanner = new Scanner(logFile);
			
			// Read starting position and alliance color
			String position = scanner.next(), alliance = scanner.next();
			
			AutoDistances.updateAutoDistances();
			
			// Starting x is constant, but starting y depends on robot's starting position
			if (alliance.equals("BLUE")) {
				switch (position) {
				case "LEFT":
					startY = converted(AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftCornerOffset - Constants.kRobotWidthInches / 2.0, false);
					break;
				case "CENTER":
					startY = converted(AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftToCenterY - Constants.kRobotWidthInches / 2.0, false);
					break;
				case "RIGHT":
					startY = converted(AutoDistances.blue.kRightCornerOffset + Constants.kRobotWidthInches / 2.0, false);
					break;
				}
			} else {
				switch (position) {
				case "LEFT":
					startY = converted(AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftCornerOffset - Constants.kRobotWidthInches / 2.0, false);
					break;
				case "CENTER":
					startY = converted(AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftToCenterY - Constants.kRobotWidthInches / 2.0, false);
					break;
				case "RIGHT":
					startY = converted(AutoDistances.red.kRightCornerOffset + Constants.kRobotWidthInches / 2.0, false);
					break;
				}
			}
			
			// Read in the waypoints of the path in the form of paired x, y coordinates
			int numWaypoints = scanner.nextInt();
			ArrayList<Waypoint> waypoints = new ArrayList<>();
			// Since a constructed path deletes the first waypoint, we circumvent this by duplicating it
			waypoints.add(new Waypoint(new Translation2d(0, 0), 0)); 
			for (int i = 0; i < numWaypoints; i++) {
				waypoints.add(new Waypoint(new Translation2d(scanner.nextDouble(), scanner.nextDouble()), 0));
			}
			setPath(new Path(waypoints));
			
			// Read in the sequence of pose estimations, the length of which is unknown
			while (scanner.hasNext()) {
				xPositions.add(scanner.nextDouble());
				yPositions.add(scanner.nextDouble());
				headings.add(scanner.nextDouble());
				xLookaheads.add(scanner.nextDouble());
				yLookaheads.add(scanner.nextDouble());
			}
			scanner.close();
			
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}
	
	// Called at instantiation: draws a basic Power Up field based on the deployed auto distances
	private void drawField() {
		GraphicsContext graphics = canvas.getGraphicsContext2D();
		
		// Since the positive y direction is down in JavaFX, y-coordinate calculations get nasty fast
		if (AutoModeBase.mAlliance == Alliance.BLUE) {
			graphics.setStroke(new Color(0, 0, 1, 1));
			graphics.setLineWidth(1.0);
			
			// Draw entire switch
			graphics.strokePolygon(converted(new double[] {AutoDistances.blue.kLeftSwitchX, AutoDistances.blue.kRightSwitchX, AutoDistances.blue.kRightSwitchX + AutoDistances.blue.kSwitchPlateLength, AutoDistances.blue.kLeftSwitchX + AutoDistances.blue.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftSwitchY, AutoDistances.blue.kRightSwitchY, AutoDistances.blue.kRightSwitchY, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftSwitchY}, false), 4);
			// Draw entire scale
			graphics.strokePolygon(converted(new double[] {AutoDistances.blue.kLeftScaleX, AutoDistances.blue.kRightScaleX, AutoDistances.blue.kRightScaleX + AutoDistances.blue.kScalePlateLength, AutoDistances.blue.kLeftScaleX + AutoDistances.blue.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftScaleY, AutoDistances.blue.kRightScaleY, AutoDistances.blue.kRightScaleY, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftScaleY}, false)  , 4);
			
			graphics.setLineWidth(3.0);
			
			// Draw left switch plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.blue.kLeftSwitchX, AutoDistances.blue.kLeftSwitchX, AutoDistances.blue.kLeftSwitchX + AutoDistances.blue.kSwitchPlateLength, AutoDistances.blue.kLeftSwitchX + AutoDistances.blue.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftSwitchY, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftSwitchY - AutoDistances.blue.kSwitchPlateWidth, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftSwitchY - AutoDistances.blue.kSwitchPlateWidth, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftSwitchY}, false), 4);
			// Draw right switch plate 
			graphics.strokePolygon(converted(new double[] {AutoDistances.blue.kRightSwitchX, AutoDistances.blue.kRightSwitchX, AutoDistances.blue.kRightSwitchX + AutoDistances.blue.kSwitchPlateLength, AutoDistances.blue.kRightSwitchX + AutoDistances.blue.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.blue.kRightSwitchY + AutoDistances.blue.kSwitchPlateWidth, AutoDistances.blue.kRightSwitchY, AutoDistances.blue.kRightSwitchY, AutoDistances.blue.kRightSwitchY + AutoDistances.blue.kSwitchPlateWidth}, false), 4);
		
			// Draw left scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.blue.kLeftScaleX, AutoDistances.blue.kLeftScaleX, AutoDistances.blue.kLeftScaleX + AutoDistances.blue.kScalePlateLength, AutoDistances.blue.kLeftScaleX + AutoDistances.blue.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftScaleY, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftScaleY - AutoDistances.blue.kScalePlateWidth, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftScaleY - AutoDistances.blue.kScalePlateWidth, AutoDistances.blue.kFieldWidth - AutoDistances.blue.kLeftScaleY}, false), 4);
			// Draw right scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.blue.kRightScaleX, AutoDistances.blue.kRightScaleX, AutoDistances.blue.kRightScaleX + AutoDistances.blue.kScalePlateLength, AutoDistances.blue.kRightScaleX + AutoDistances.blue.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.blue.kRightScaleY + AutoDistances.blue.kScalePlateWidth, AutoDistances.blue.kRightScaleY, AutoDistances.blue.kRightScaleY, AutoDistances.blue.kRightScaleY + AutoDistances.blue.kScalePlateWidth}, false), 4);
			
			// Draw pyramid square
			graphics.strokePolygon(converted(new double[] {AutoDistances.blue.kLeftSwitchX - AutoDistances.blue.kPyramidLength, AutoDistances.blue.kLeftSwitchX - AutoDistances.blue.kPyramidLength, AutoDistances.blue.kLeftSwitchX, AutoDistances.blue.kLeftSwitchX}, true),
									converted(new double[] {AutoDistances.blue.kPyramidFromRightY + AutoDistances.blue.kPyramidWidth, AutoDistances.blue.kPyramidFromRightY, AutoDistances.blue.kPyramidFromRightY, AutoDistances.blue.kPyramidFromRightY + AutoDistances.blue.kPyramidWidth}, false), 4);
		} else {
			graphics.setStroke(new Color(1, 0, 0, 1));
			graphics.setLineWidth(1.0);
			
			// Draw entire switch
			graphics.strokePolygon(converted(new double[] {AutoDistances.red.kLeftSwitchX, AutoDistances.red.kRightSwitchX, AutoDistances.red.kRightSwitchX + AutoDistances.red.kSwitchPlateLength, AutoDistances.red.kLeftSwitchX + AutoDistances.red.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftSwitchY, AutoDistances.red.kRightSwitchY, AutoDistances.red.kRightSwitchY, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftSwitchY}, false), 4);
			// Draw entire scale
			graphics.strokePolygon(converted(new double[] {AutoDistances.red.kLeftScaleX, AutoDistances.red.kRightScaleX, AutoDistances.red.kRightScaleX + AutoDistances.red.kScalePlateLength, AutoDistances.red.kLeftScaleX + AutoDistances.red.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftScaleY, AutoDistances.red.kRightScaleY, AutoDistances.red.kRightScaleY, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftScaleY}, false)  , 4);
			
			graphics.setLineWidth(3.0);
			
			// Draw left switch plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.red.kLeftSwitchX, AutoDistances.red.kLeftSwitchX, AutoDistances.red.kLeftSwitchX + AutoDistances.red.kSwitchPlateLength, AutoDistances.red.kLeftSwitchX + AutoDistances.red.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftSwitchY, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftSwitchY - AutoDistances.red.kSwitchPlateWidth, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftSwitchY - AutoDistances.red.kSwitchPlateWidth, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftSwitchY}, false), 4);
			// Draw right switch plate 
			graphics.strokePolygon(converted(new double[] {AutoDistances.red.kRightSwitchX, AutoDistances.red.kRightSwitchX, AutoDistances.red.kRightSwitchX + AutoDistances.red.kSwitchPlateLength, AutoDistances.red.kRightSwitchX + AutoDistances.red.kSwitchPlateLength}, true),
									converted(new double[] {AutoDistances.red.kRightSwitchY + AutoDistances.red.kSwitchPlateWidth, AutoDistances.red.kRightSwitchY, AutoDistances.red.kRightSwitchY, AutoDistances.red.kRightSwitchY + AutoDistances.red.kSwitchPlateWidth}, false), 4);
		
			// Draw left scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.red.kLeftScaleX, AutoDistances.red.kLeftScaleX, AutoDistances.red.kLeftScaleX + AutoDistances.red.kScalePlateLength, AutoDistances.red.kLeftScaleX + AutoDistances.red.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftScaleY, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftScaleY - AutoDistances.red.kScalePlateWidth, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftScaleY - AutoDistances.red.kScalePlateWidth, AutoDistances.red.kFieldWidth - AutoDistances.red.kLeftScaleY}, false), 4);
			// Draw right scale plate
			graphics.strokePolygon(converted(new double[] {AutoDistances.red.kRightScaleX, AutoDistances.red.kRightScaleX, AutoDistances.red.kRightScaleX + AutoDistances.red.kScalePlateLength, AutoDistances.red.kRightScaleX + AutoDistances.red.kScalePlateLength}, true),
									converted(new double[] {AutoDistances.red.kRightScaleY + AutoDistances.red.kScalePlateWidth, AutoDistances.red.kRightScaleY, AutoDistances.red.kRightScaleY, AutoDistances.red.kRightScaleY + AutoDistances.red.kScalePlateWidth}, false), 4);
		
			// Draw pyramid square
			graphics.strokePolygon(converted(new double[] {AutoDistances.red.kLeftSwitchX - AutoDistances.red.kPyramidLength, AutoDistances.red.kLeftSwitchX - AutoDistances.red.kPyramidLength, AutoDistances.red.kLeftSwitchX, AutoDistances.red.kLeftSwitchX}, true),
												converted(new double[] {AutoDistances.red.kPyramidFromRightY + AutoDistances.red.kPyramidWidth, AutoDistances.red.kPyramidFromRightY, AutoDistances.red.kPyramidFromRightY, AutoDistances.red.kPyramidFromRightY + AutoDistances.red.kPyramidWidth}, false), 4);
		}
	}
	
	// Called at instantiation: draws the waypoints and segments of the path being run
	private void drawPath() {
		GraphicsContext graphics = canvas.getGraphicsContext2D();
		graphics.setStroke(new Color(0, 0, 0, 1));
		graphics.setLineWidth(10.0);
		for (Waypoint point : mPath.getWaypoints()) {
			double x = startX + point.position.getX() * canvasScale, y = startY - point.position.getY() * canvasScale;
			graphics.fillOval(x - graphics.getLineWidth() / 2.0, y - graphics.getLineWidth() / 2.0, graphics.getLineWidth(), graphics.getLineWidth());
		}
		
		graphics.setLineWidth(1.0);
		for (PathSegment segment : mPath.getSegments()) {
			double x1 = startX + segment.getStart().getX() * canvasScale, y1 = startY - segment.getStart().getY() * canvasScale;
			double x2 = startX + segment.getEnd().getX() * canvasScale, y2 = startY - segment.getEnd().getY() * canvasScale;
			graphics.strokeLine(x1, y1, x2, y2);
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
	
	private static double converted(double coordinate, boolean isXCoordinate) {
		double retval = 0.0;
		if (isXCoordinate) {
			retval = canvasScale * coordinate;
		} else {
			retval = canvasLength - canvasScale * coordinate;
		}
		return retval;
	}
	
}
