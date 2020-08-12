package org.firstinspires.ftc.teamcode.rework.Robot.Auto.PathPlanning;

public class Waypoint extends Point {
    boolean actionable;
    Actions action;

    public Waypoint(double x, double y){
        super(x,y);
        actionable = false;
    }

    public Waypoint(double x, double y, Actions action) {
        super(x,y);
        actionable = true;
        this.action = action;
    }
}
