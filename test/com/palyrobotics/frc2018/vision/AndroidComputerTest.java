package com.palyrobotics.frc2018.vision;

import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.logger.Logger;

import java.io.File;

public class AndroidComputerTest {

    public static void main(String[] args) {
        Logger.getInstance().setFileName("Android Test");
        Logger.getInstance().start();
	    System.out.print(new File("src/com/palyrobotics/frc2018/vision/util/default.jpeg").getAbsolutePath());
	    System.out.print(new File("src/com/palyrobotics/frc2018/vision/util/default.jpeg").exists());

        VisionManager.getInstance().start(Constants.kVisionManagerUpdateRate, true);
    }
}
