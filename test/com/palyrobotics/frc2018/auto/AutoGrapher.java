package com.palyrobotics.frc2018.auto;

import com.palyrobotics.frc2018.auto.modes.CenterStartLeftSwitchAutoMode;
import com.palyrobotics.frc2018.auto.modes.CenterStartRightSwitchAutoMode;
import com.palyrobotics.frc2018.auto.modes.LeftStartLeftSwitchAutoMode;
import com.palyrobotics.frc2018.auto.modes.RightStartLeftSwitchAutoMode;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.config.fields.FieldSelector;
import org.junit.Test;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;

public class AutoGrapher {

    @Test
    public void showAuto() throws IOException {
        FieldSelector.configureFieldMeasurements();
        for (int i = 0; i < 26; i+=2) {
            loadAutos(AutoModeSelector.getInstance().getAutoModeByIndex(i));
        }

        Runtime.getRuntime().exec( new String[] { "open" , "-a", "Safari", System.getProperty("user.dir") + "/auto-graph/index.html"}) ;
    }

    public void loadAutos(AutoModeBase auto) throws IOException {
        AutoModeBase base = auto;
        base.prestart();

        String pathType = base.getClass().getName().split("\\.")[base.getClass().getName().split("\\.").length-1];
        System.out.println("Loading " + pathType);
        FileOutputStream out = new FileOutputStream("auto-graph/"+pathType + ".txt");
        ArrayList<Routine> pathExport = base.getRoutine().getEnclosingSequentialRoutine();
        for (int i = 0; i < pathExport.size(); i++) {
            if (pathExport.get(i) instanceof DrivePathRoutine) {
                out.write(pathExport.get(i).toString().getBytes());
            }
        }
        out.close();
    }
}
