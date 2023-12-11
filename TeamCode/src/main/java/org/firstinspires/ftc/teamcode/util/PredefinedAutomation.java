package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;

public final class PredefinedAutomation {
    public Waypoint[] test = new Waypoint[] {
            new StartWaypoint(0, 0),
            new GeneralWaypoint(100, 100),
            new EndWaypoint()
    };
}
