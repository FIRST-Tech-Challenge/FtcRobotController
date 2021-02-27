package org.firstinspires.ftc.teamcode.toolkit.misc;

public class PathPoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double errorDistance;

    public PathPoint(double x, double y, double moveSpeed, double errorDistance) {

        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.errorDistance = errorDistance;
    }

    public PathPoint(PathPoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        errorDistance = thisPoint.errorDistance;
    }

    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
    }
}
