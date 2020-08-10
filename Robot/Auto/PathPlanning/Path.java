package org.firstinspires.ftc.teamcode.rework.Robot.Auto.PathPlanning;

import java.util.HashMap;

public class Path {
    HashMap<Point, Actions> path;

    Path(HashMap<Point, Actions> path) {
        this.path = path;
    }
}
