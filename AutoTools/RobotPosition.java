package org.firstinspires.ftc.teamcode.rework.AutoTools;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;

public class RobotPosition {

    private Point location;
    private double heading;

    public RobotPosition(){
        this.location = new Point();
        this.heading = 0;
    }

    public RobotPosition(Point location, double heading) {
        this.location = location;
        this.heading = heading;
    }

    public void updatePosition(double x, double y, double heading) {
        this.location = new Point(x,y);
        this.heading = heading;
    }

    public Point getLocation() {
        return location;
    }

    public double getHeading() {
        return heading;
    }
}
