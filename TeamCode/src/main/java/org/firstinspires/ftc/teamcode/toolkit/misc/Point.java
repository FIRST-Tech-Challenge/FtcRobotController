package org.firstinspires.ftc.teamcode.toolkit.misc;

public class Point {
    public double x;
    public double y;
    public double angle;

    public Point() {
        x = 0;
        y = 0;
        angle = 0;
    }

    public Point(double xCoord, double yCoord, double angleCoord) {
        x = xCoord;
        y = yCoord;
        angle = angleCoord;
    }

    public Point(Point p) {
        x = p.x;
        y = p.y;
    }

}
