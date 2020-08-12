package org.firstinspires.ftc.teamcode.rework.Robot.Auto.PathPlanning;

public class Waypoint extends Point {
    Actions[] actions;

    public Waypoint(double x, double y){
        super(x,y);
    }

    public Waypoint(double x, double y, Actions[] actions) {
        super(x,y);
        this.actions = actions;
    }
}
