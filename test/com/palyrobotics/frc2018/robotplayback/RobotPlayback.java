package com.palyrobotics.frc2018.robotplayback;

import static java.time.temporal.ChronoField.HOUR_OF_DAY;
import static java.time.temporal.ChronoField.MINUTE_OF_HOUR;
import static java.time.temporal.ChronoField.NANO_OF_SECOND;
import static java.time.temporal.ChronoField.SECOND_OF_MINUTE;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeFormatterBuilder;
import java.time.format.ResolverStyle;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeMap;
import java.util.TreeSet;

import org.apache.commons.lang3.ArrayUtils;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.auto.AutoModeBase.Alliance;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.panayotis.gnuplot.JavaPlot;
import com.panayotis.gnuplot.dataset.ArrayDataSet;
import com.panayotis.gnuplot.plot.DataSetPlot;
import com.panayotis.gnuplot.style.PlotStyle;
import com.panayotis.gnuplot.style.Style;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.VPos;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TabPane.TabClosingPolicy;
import javafx.scene.control.TextArea;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.RowConstraints;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.text.Text;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;

public class RobotPlayback extends Application {
	
	// The sequence of estimated poses split into individual components
	
	private Group root;
	private BorderPane border;
	private Scene scene;
	private Canvas background;
	private Canvas path;
	private Slider slider;
	private Text status;
	private TextArea log;
	private StackPane center;
	
	
	private TreeMap<LocalTime, HashMap<String, String>> parsedFile;
	private TreeSet<String> keys;
	private ArrayList<RecordedPath> paths;
	private String logText = "";
	
	// Has playback been paused or not?
	private boolean paused = false;
	private boolean update = true;
	
	// What point in the list of estimated positions are we displaying?
	
	// Starting position of the robot as determined by AutoModeBase
	private static double startX = converted(Constants.kRobotLengthInches - Constants.kCenterOfRotationOffsetFromFrontInches, true);
	private static double startY = 0;
	
	private long currentTime = 0;
	private long endTime;
	
	private static final double canvasScale = 2.6;
	private static final double canvasWidth = 400 * canvasScale;
	private static final double canvasLength = 324 * canvasScale;
	
	public static final DateTimeFormatter TIME_FORMAT;
    static {
        TIME_FORMAT = new DateTimeFormatterBuilder()
                .appendValue(HOUR_OF_DAY, 2)
                .appendLiteral(':')
                .appendValue(MINUTE_OF_HOUR, 2)
                .optionalStart()
                .appendLiteral(':')
                .appendValue(SECOND_OF_MINUTE, 2)
                .optionalStart()
                .appendFraction(NANO_OF_SECOND, 3, 3, true)
                .toFormatter();
    }
	

	public void start(Stage stage) throws Exception {
		
		handleInput(stage);
		
		endTime = ChronoUnit.NANOS.between(parsedFile.firstKey(), parsedFile.lastKey());
		
		stage.setTitle("Robot Playback");
		
		//Playback tab
		
		border = new BorderPane();
				
		root = new Group();
		background = new Canvas(canvasWidth, canvasLength);
		path = new Canvas(canvasWidth, canvasLength);
		background.getGraphicsContext2D().strokeLine(canvasWidth, 0, canvasWidth, canvasLength);
		root.getChildren().add(background);
		center = new StackPane();
		center.setMinSize(StackPane.USE_PREF_SIZE, StackPane.USE_PREF_SIZE);
		center.setPrefSize(canvasWidth, canvasLength);
		center.setMaxSize(StackPane.USE_PREF_SIZE, StackPane.USE_PREF_SIZE);
		center.getChildren().add(root);
		border.setCenter(center);

		slider = new Slider(0, (double)endTime/1e9, 0);
		slider.setShowTickMarks(true);
		slider.setShowTickLabels(true);
		slider.setMajorTickUnit(2);
		border.setBottom(slider);
		
		status = new Text();
		GridPane.setMargin(status, new Insets(10, 10, 10, 10));
		GridPane.setRowIndex(status, 0);
	    GridPane.setColumnIndex(status, 0);
		GridPane.setValignment(status, VPos.TOP);
	    
	    log = new TextArea(logText);
	    log.setStyle("-fx-font-size: .85em;");
	    log.setPrefWidth(200*canvasScale);
	    log.setEditable(false);
	   

	    GridPane.setRowIndex(log, 1);
	    GridPane.setColumnIndex(log, 0);
	    
		GridPane right = new GridPane();
		
		right.setPrefWidth(200*canvasScale);
		right.setMaxWidth(200*canvasScale);
		RowConstraints row1 = new RowConstraints();
		row1.setPercentHeight(25);
		RowConstraints row2 = new RowConstraints();
		row2.setPercentHeight(75);
		right.getRowConstraints().addAll(row1, row2);
		right.getChildren().add(status);
		right.getChildren().add(log);
		
		border.setRight(right);
		
		Tab playbackTab = new Tab();
		playbackTab.setText("Playback");
		playbackTab.setContent(border);
		
		//Export tab
		
		VBox layout = new VBox();
		ArrayList<CheckBox> boxes = new ArrayList<CheckBox>();
		
		for (String s: keys) {
			CheckBox box = new CheckBox(s);
			boxes.add(box);
		}
		
		layout.getChildren().addAll(boxes);
		
		Button export = new Button();
		export.setText("Export");
		
		layout.getChildren().add(export);
		
		export.setOnAction(new EventHandler<ActionEvent>() {

			@Override
			public void handle(ActionEvent event) {
				
				FileChooser fileChooser = new FileChooser();
				fileChooser.getExtensionFilters().add(new ExtensionFilter("CSV", "*.csv"));
				fileChooser.setTitle("Select a file to export to");
				fileChooser.setInitialFileName("data.csv");
				File exportFile = fileChooser.showSaveDialog(stage);
				
				TreeSet<String> export_keys = new TreeSet<String>();
				
				for(CheckBox b: boxes) {
					if (b.isSelected()) {
						export_keys.add(b.getText());
					}
				}
				
				try {
					BufferedWriter writer = new BufferedWriter(new FileWriter(exportFile));
					
					String header = "timestamp,";
					for(String s: export_keys) {
						header+=s+",";
					}
					
					writer.write(header);
					writer.newLine();
					
					for (LocalTime time: parsedFile.keySet()) {
						HashMap<String, String> line = parsedFile.get(time);
						
						String l = String.format("%.5f",ChronoUnit.NANOS.between(parsedFile.firstKey(), time)/(1.0e9)) + ",";
						
						for (String s: export_keys) {
							if (line.containsKey(s)) {
								l+=line.get(s)+",";
							}
							else {
								l+=",";
							}
						}
						writer.write(l);
						writer.newLine();
					}
					
					writer.close();
					
				}
				catch (IOException e) {
					e.printStackTrace();
				}
			}
			
		});
		
		Tab exportTab = new Tab();
		exportTab.setText("Export");
		exportTab.setContent(layout);
		
		//Plot tab
		
		VBox plot_layout = new VBox();
				
		ArrayList<CheckBox> plot_boxes = new ArrayList<CheckBox>();
		HashMap<String, XYChart.Series<Number, Number>> keyToData = new HashMap<String, XYChart.Series<Number, Number>>();
		
		for (String s: keys) {
			CheckBox box = new CheckBox(s);
			plot_boxes.add(box);

			XYChart.Series<Number, Number> series = new XYChart.Series<Number, Number>();
			series.setName(s);

			for (LocalTime time: parsedFile.keySet()) {
				HashMap<String, String> line = parsedFile.get(time);

				double value;

				try {
					value = Double.parseDouble(line.get(s)); 
				}
				catch (Exception e) {
					value = 0;
				}

				series.getData().add(new XYChart.Data<Number, Number>(ChronoUnit.NANOS.between(parsedFile.firstKey(), time)/(1.0e9), value));

			}

			keyToData.put(s, series);			

		}
		
		plot_layout.getChildren().addAll(plot_boxes);
		
		Button reset = new Button();
		reset.setText("Reset");
		
		Button gnuplot = new Button();
		gnuplot.setText("Plot in GNUPlot");
		
		plot_layout.getChildren().addAll(reset, gnuplot);
		
		
		final NumberAxis xAxis = new NumberAxis();
        final NumberAxis yAxis = new NumberAxis();
        xAxis.setLabel("Time (s)");
        yAxis.setLabel("Data");
        
        LineChart<Number,Number> lineChart = new LineChart<Number,Number>(xAxis,yAxis);
        lineChart.setCreateSymbols(false);
        lineChart.setAnimated(false);
        lineChart.setVerticalGridLinesVisible(true);
        lineChart.setHorizontalGridLinesVisible(true);
        lineChart.setVerticalZeroLineVisible(true);
        lineChart.setHorizontalZeroLineVisible(true);
		
		plot_layout.getChildren().add(lineChart);
		
		reset.setOnAction(new EventHandler<ActionEvent>() {

			@Override
			public void handle(ActionEvent event) {
				for (CheckBox b: plot_boxes) {
					b.setSelected(false);
				}
				lineChart.getData().clear();
				
			}
			
		});
		
		gnuplot.setOnAction(new EventHandler<ActionEvent>() {

			@Override
			public void handle(ActionEvent event) {
				JavaPlot plot = new JavaPlot();
				for (CheckBox b: plot_boxes) {
					if (b.isSelected()) {
						String s = b.getText();
						
						double[][] values = new double[parsedFile.size()][2];
						int i = 0;
						
						for (LocalTime time: parsedFile.keySet()) {
							HashMap<String, String> line = parsedFile.get(time);

							double value;

							try {
								value = Double.parseDouble(line.get(s)); 
							}
							catch (Exception e) {
								value = 0;
							}

							values[i][0] = ChronoUnit.NANOS.between(parsedFile.firstKey(), time)/(1.0e9);
							values[i][1] = value;
							
							i++;
						}
						DataSetPlot ds = new DataSetPlot(values);
						ds.setTitle(s.replace("_", "\\_"));
						PlotStyle ps = new PlotStyle();
						ps.setStyle(Style.LINES);
						ds.setPlotStyle(ps);
						plot.addPlot(ds);
					}
				}
				plot.plot();
			}
			
		});
				
		EventHandler<ActionEvent> plot_handler = new EventHandler<ActionEvent>() {

			@Override
			public void handle(ActionEvent event) {
				
				for (CheckBox b: plot_boxes) {
					if (b.isSelected() && !lineChart.getData().contains(keyToData.get(b.getText()))) {
						lineChart.getData().add(keyToData.get(b.getText()));
					}
					else if (!b.isSelected() && lineChart.getData().contains(keyToData.get(b.getText()))) {
						lineChart.getData().remove(keyToData.get(b.getText()));
					}
				}
				
			}
			
		};
		
		for(CheckBox b: plot_boxes) {
			b.setOnAction(plot_handler);
		}
		
		
		Tab plotTab = new Tab();
		plotTab.setText("Plot");
		plotTab.setContent(plot_layout);
		
		//Set up scene
		
		TabPane tabs = new TabPane();
		tabs.setTabClosingPolicy(TabClosingPolicy.UNAVAILABLE);
		
		tabs.getTabs().addAll(playbackTab, exportTab, plotTab);
		
		scene = new Scene(tabs);
		stage.setScene(scene);
		
		new AnimationTimer() {

			@Override
			public void handle(long arg0) {
				
				if (tabs.getSelectionModel().getSelectedItem() == playbackTab) {
					border.requestFocus();
				}
				
				if (!paused || update) {
					
					LocalTime currentLocalTime = parsedFile.firstKey().plus(currentTime, ChronoUnit.NANOS);
					
					HashMap<String, String> line = (HashMap<String, String>) parsedFile.floorEntry(currentLocalTime).getValue();
					Scale scale = new Scale(canvasScale, canvasScale, 0, 0);
					
					root = new Group();
					root.getChildren().add(background);
					
					boolean drawEstimatedRobot = false;
					
					for(RecordedPath p: paths) {
						if (currentLocalTime.isAfter(p.timestamps.get(0)) && currentLocalTime.isBefore(p.timestamps.get(p.timestamps.size()-1))) {
							drawPath(p);
							root.getChildren().add(path);
							drawEstimatedRobot = true;
						}
					}
					
					//Draw the robot at its estimated position
					Rectangle actual_robot = new Rectangle(-Constants.kRobotLengthInches + Constants.kCenterOfRotationOffsetFromFrontInches, -Constants.kRobotWidthInches / 2.0, Constants.kRobotLengthInches, Constants.kRobotWidthInches);
					Circle actual_robotCenter = new Circle(0, 0, 2.0);
					
					Rotate actual_rotation = new Rotate(-parseDouble(line, "robot_heading"), 0, 0);
					Translate actual_translation = new Translate(startX + parseDouble(line, "robot_x") * canvasScale, startY - parseDouble(line, "robot_y") * canvasScale);
					
					// NOTE: The transformations are applied in the reverse of the order you add them in!
					actual_robot.getTransforms().addAll(actual_translation, actual_rotation, scale);
					actual_robot.setStroke(new Color(0, 1, 0, 1));
					actual_robot.setFill(new Color(1, 1, 1, 0));
					
					actual_robotCenter.getTransforms().addAll(actual_translation, actual_rotation, scale);
					actual_robotCenter.setFill(new Color(1, 0, 0, 1));
					
					root.getChildren().add(actual_robot);
					root.getChildren().add(actual_robotCenter);
					
					
//					if (drawEstimatedRobot) {
//
//						Rectangle estimated_robot = new Rectangle(-Constants.kRobotLengthInches + Constants.kCenterOfRotationOffsetFromFrontInches, -Constants.kRobotWidthInches / 2.0, Constants.kRobotLengthInches, Constants.kRobotWidthInches);
//						Circle estimated_robotCenter = new Circle(0, 0, 2.0);
//						
//						Rotate estimated_rotation = new Rotate(-parseDouble(line, "estimated_robot_heading"), 0, 0);
//						Translate estimated_translation = new Translate(startX + parseDouble(line, "estimated_robot_x") * canvasScale, startY - parseDouble(line, "estimated_robot_y") * canvasScale);
//						
//						// NOTE: The transformations are applied in the reverse of the order you add them in!
//						estimated_robot.getTransforms().addAll(estimated_translation, estimated_rotation, scale);
//						estimated_robot.setStroke(new Color(0, 0, 0, 0.2));
//						estimated_robot.setFill(new Color(1, 1, 1, 0));
//						
//						estimated_robotCenter.getTransforms().addAll(estimated_translation, estimated_rotation, scale);
//						estimated_robotCenter.setFill(new Color(0, 0, 0, 0.2));
//						
//						root.getChildren().add(estimated_robot);
//						root.getChildren().add(estimated_robotCenter);
//					}
					
					
//					root.getChildren().add(lookaheadLine);
//					root.getChildren().add(lookaheadPoint);
					
					root.setManaged(false);
					center.getChildren().setAll(root);
										
					border.requestFocus();
					
					
					status.setText("Current Timestamp: " + currentLocalTime.format(TIME_FORMAT) + "\n" +
							"Current Time: " + String.format("%.3f", ChronoUnit.NANOS.between(parsedFile.firstKey(), currentLocalTime)/(1.0e9)) + "s\n" +
							"Elevator Height: " + String.format( "%.2f", parseDouble(line, "elevator_position")) + "\n" +
							"Intake Wheel State: " + line.get("intake_wheel") + "\n" +
							"Intake Open Close State: " + line.get("intake_open_close") + "\n" +
							"Intake Up Down State: " + line.get("intake_up_down"));

					if (currentTime < endTime && !paused) {
						currentTime += (long)2e7;
						slider.setValue(currentTime/1e9);
					}
					else if (currentTime >= endTime && !paused) {
						slider.setValue(currentTime/1e9);
						paused = true;
					}
					
					update = false;
					
				}
			}
			
		}.start();
		
		// Configure keyboard control to stop, start, increment, and rewind playback
		border.setOnKeyPressed(new EventHandler<KeyEvent>() {

			@Override
			public void handle(KeyEvent event) {
				switch (event.getCode()) {
				
				case SPACE:
					paused = !paused;
					break;
				case LEFT:
					if (paused) {
						currentTime = Math.max(currentTime - (int)1e8, 0);
						slider.setValue(currentTime/1e9);
						update = true;
					}
					break;
				case RIGHT:
					if (paused) {
						currentTime = Math.min(currentTime + (int)1e8, endTime);
						slider.setValue(currentTime/1e9);
						update = true;
					}
					break;
				case HOME:
					if (paused) {
						currentTime = 0;
						slider.setValue(0);
						update = true;
					}
				default:
					break;
				
				}
			}
			
		});
		
		slider.valueProperty().addListener(new ChangeListener<Number>() {
			@Override
			public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
				currentTime = (long)(newValue.doubleValue()*1e9);
				update = true;
			}
        });
		
		drawField();
		
		stage.show();
	}
	
	public static void main(String[] args) {
		launch(args);
	}
	
	// Parse the auto log file selected by the user
	private void handleInput(Stage stage) {
		FileChooser fileChooser = new FileChooser();
		fileChooser.getExtensionFilters().add(new ExtensionFilter("Robot Data Log Files", "*.datalog"));
		fileChooser.setTitle("Select an data log file to visualize");
		File dataLogFile = fileChooser.showOpenDialog(stage);
		
		try {
			File logFile = new File(dataLogFile.getAbsolutePath().replaceAll(".datalog", ".log"));
			BufferedReader reader = new BufferedReader(new FileReader(logFile));
			String currentLine;
			while ((currentLine = reader.readLine()) != null) {
				logText += currentLine+"\n";
			}
			reader.close();
		}
		catch (Exception e) {
			logText="No log file of same name found";
		}
		
		
		parsedFile = parseFile(dataLogFile);
		keys = getKeys(parsedFile);
		paths = readPaths(parsedFile);
//		for (RecordedPath p: paths) {
//			p.print();
//		}
			RecordedPath path = paths.get(0);
			
			String alliance = path.alliance;
			String position = path.start_location;
			
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
			
		

	}
	
	// Called at instantiation: draws a basic Power Up field based on the deployed auto distances
	private void drawField() {
		GraphicsContext graphics = background.getGraphicsContext2D();
		
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
	private void drawPath(RecordedPath p) {
		GraphicsContext graphics = path.getGraphicsContext2D();
		graphics.clearRect(0, 0, path.getWidth(), path.getHeight());
		graphics.setStroke(new Color(0, 0, 0, 1));
		
		
		graphics.setLineWidth(3.0);
		for (int i = 1; i < p.waypoints; i++) {
			double x1 = startX + p.waypointsX[i-1] * canvasScale, y1 = startY - p.waypointsY[i-1] * canvasScale;
			double x2 = startX + p.waypointsX[i] * canvasScale, y2 = startY - p.waypointsY[i] * canvasScale;
			graphics.strokeLine(x1, y1, x2, y2);
		}
		
		graphics.setFill(new Color(0, 0, 1, 1));
		graphics.setLineWidth(5.0);
		for (int i = 0; i < p.waypoints; i++) {
			double x = startX + p.waypointsX[i] * canvasScale, y = startY - p.waypointsY[i] * canvasScale;
			graphics.fillOval(x - graphics.getLineWidth() / 2.0, y - graphics.getLineWidth() / 2.0, graphics.getLineWidth(), graphics.getLineWidth());
		}
		
		graphics.setStroke(new Color(0, 1, 1, 0.5));
		graphics.setLineWidth(3.0);
		
		double[] xPos = ArrayUtils.toPrimitive(p.xPositions.toArray(new Double[0]));
		double[] yPos = ArrayUtils.toPrimitive(p.yPositions.toArray(new Double[0]));
		
		for (int i = 0; i < xPos.length; i++) {
			xPos[i] = startX + xPos[i]*canvasScale;
			yPos[i] = startY - yPos[i]*canvasScale;
		}
		
		graphics.strokePolyline(xPos, yPos, xPos.length);
	
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
	
	private static TreeSet<String> getKeys(TreeMap<LocalTime, HashMap<String, String>> parsedLog) {
		TreeSet<String> keys = new TreeSet<String>();
		for (LocalTime time: parsedLog.keySet()) {
			HashMap<String, String> line = parsedLog.get(time);
			keys.addAll(line.keySet());
		}
		return keys;
	}
	
	private static ArrayList<RecordedPath> readPaths(TreeMap<LocalTime, HashMap<String, String>> parsedLog) {
		
		ArrayList<RecordedPath> paths = new ArrayList<RecordedPath>();
		boolean onPath = false;
		RecordedPath activePath = null;
					
		for (LocalTime time: parsedLog.keySet()) {
			HashMap<String, String> line = parsedLog.get(time);
			
			if (line.containsKey("path_metadata")) {
				String path_metadata = line.get("path_metadata");
				String[] split_metadata = path_metadata.split("/|];");
				String start_location = split_metadata[0];
				String alliance = split_metadata[1];
				int waypoints = Integer.parseInt(split_metadata[2]);
				double[] waypointsX = new double[waypoints];
				double[] waypointsY = new double[waypoints];
				for (int i = 3; i < split_metadata.length; i++) {
					waypointsX[i-3] = Double.parseDouble(split_metadata[i].split(";")[0].replace("[", " "));
					waypointsY[i-3] = Double.parseDouble(split_metadata[i].split(";")[1]);
				}
				activePath = new RecordedPath(alliance, start_location, waypoints, waypointsX, waypointsY);
				paths.add(activePath);
				onPath = true;
			}
			else if (line.containsKey("path_end")) {
				activePath = null;
				onPath = false;
			}
			
			if (onPath) {
				activePath.addPoint(time, Double.parseDouble(line.get("estimated_robot_x")), Double.parseDouble(line.get("estimated_robot_y")),
						Double.parseDouble(line.get("estimated_robot_heading")), Double.parseDouble(line.get("lookahead_x")), Double.parseDouble(line.get("lookahead_y")));
			}
			
		}
			
		
		return paths;
	}
	
	private static TreeMap<LocalTime, HashMap<String, String>> parseFile(File logfile) {
		try {
			BufferedReader reader = new BufferedReader(new FileReader(logfile));
			TreeMap<LocalTime, HashMap<String, String>> parsedLog = new TreeMap<LocalTime, HashMap<String, String>>();
			int lineCount = 1;
			String line;
			while ((line = reader.readLine()) != null) {
				if (lineCount < 3) {
					lineCount++;
					continue;
				}
				String[] split_line = line.split(": |, ");
				LocalTime timestamp = LocalTime.parse(split_line[0]);
				HashMap<String, String> data = new HashMap<String, String>();
				for (int i = 1; i < split_line.length; i+=2) {
					if (i+1 >= split_line.length) {
						data.put(split_line[i], "");
					}
					else {
						data.put(split_line[i], split_line[i+1]);
					}
				}
				parsedLog.put(timestamp, data);
				lineCount++;
			}
			reader.close();
			return parsedLog;
		}
		catch (IOException e) {
			e.printStackTrace();
			return null;
		}
	}
	
	private static double parseDouble(HashMap<String, String> line, String key) {
		return Double.parseDouble(line.get(key));
	}
	private static int parseInt(HashMap<String, String> line, String key) {
		return Integer.parseInt(line.get(key));
	}
	
}
