package com.palyrobotics.frc2018.auto.modes;

import com.palyrobotics.frc2018.auto.AutoModeBase;
import com.palyrobotics.frc2018.auto.AutoModeBase.Color;
import com.palyrobotics.frc2018.auto.AutoModeBase.ScaleDecision;
import com.palyrobotics.frc2018.auto.AutoModeBase.Side;
import com.palyrobotics.frc2018.auto.AutoModeBase.SwitchDecision;
import com.palyrobotics.frc2018.auto.AutoModeBase.WhichPriority;
import com.palyrobotics.frc2018.behavior.ParallelRoutine;
import com.palyrobotics.frc2018.behavior.Routine;
import com.palyrobotics.frc2018.behavior.SequentialRoutine;
import com.palyrobotics.frc2018.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2018.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2018.config.AutoDistances;
import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.trajectory.Path;
import com.palyrobotics.frc2018.util.trajectory.Path.Waypoint;
import com.palyrobotics.frc2018.util.trajectory.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RightScaleAutoMode extends AutoModeBase {
	
	
	public RightScaleAutoMode() {

	}

    @Override
    public String toString() {
        return "Left Scale Auto Mode";
    }

    @Override
    public void prestart() {

    }

    @Override
    public Routine getRoutine() {
        List<Waypoint> initialPath = new ArrayList<>();
        initialPath.add(new Waypoint(new Translation2d(0,0), 72.0));
        if(needsToCross) {
            initialPath.add(new Waypoint(new Translation2d((AutoDistances.kDistanceToScaleStart - .5 * Constants.kRobotLengthInches - .5 * AutoDistances.kScaleToSwitchDist),
                    0.0), 72.0)); // drive to the center
            initialPath.add(new Waypoint(new Translation2d(0, -1 * AutoDistances.kDistAcrossScale), 0.0));
        }
        else {
            initialPath.add(new Waypoint(new Translation2d(0,0), 72));
            initialPath.add(new Waypoint(new Translation2d((AutoDistances.kDistanceToScaleStart - .5 * Constants.kRobotLengthInches),
                    0.0), 0)); // drive directly to the center

        }

        // path to slowly drive forward while dropping the cube
        List<Waypoint> shortDropPath = new ArrayList<Waypoint>();
        initialPath.add(new Waypoint(new Translation2d(0,0), 50));
        initialPath.add(new Waypoint(new Translation2d(12,0), 0));


        ArrayList<Routine> routines = new ArrayList<>();
        routines.add(new DriveSensorResetRoutine());
        routines.add(new ParallelRoutine(new ArrayList<Routine>() {{
            new DrivePathRoutine(new Path(initialPath), false);
            new SequentialRoutine(new ArrayList<Routine>() {{
                new TimeoutRoutine(((needsToCross ? 4 : 1.2))); // wait extra time if we need to cross
//              new RaiseElevatorRoutine());
//              new ElevatorHoldAngle());

            }});
        }}));
        routines.add(new ParallelRoutine(new ArrayList<Routine>() {{
//          routines.add(new ElevatorHoldAngle());
            new SequentialRoutine(new ArrayList<Routine>() {{
                new DrivePathRoutine(new Path(shortDropPath), false);
//              new DropRoutine()
            }});
        }}));

        return new SequentialRoutine(routines);
    }
}
