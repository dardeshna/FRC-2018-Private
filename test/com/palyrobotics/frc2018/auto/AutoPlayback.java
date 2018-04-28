package com.palyrobotics.frc2018.auto;

import com.palyrobotics.frc2018.auto.AutoModeBase.Alliance;
import com.palyrobotics.frc2018.config.AutoDistances;

import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.stage.Stage;

public class AutoPlayback extends Application {
	
	private Group root;
	private Canvas canvas;

	public void start(Stage stage) throws Exception {
		
		stage.setTitle("Auto Playback");
		
		root = new Group();
		
		Scene scene = new Scene(root);
		
		
		
	}
	
	private void drawField(Stage stage) {
		GraphicsContext graphics = canvas.getGraphicsContext2D();
		
		if (AutoModeBase.mAlliance == Alliance.BLUE) {
			graphics.setStroke(new Color(0, 0, 1, 1));
			
		} else {
			
		}
	}

}
