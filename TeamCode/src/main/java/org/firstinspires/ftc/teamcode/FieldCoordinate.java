package org.firstinspires.ftc.teamcode;

public class FieldCoordinate {
    protected double xCoordinate;
    protected double yCoordinate;
    protected double angleDegrees;
    public FieldCoordinate(double x, double y, double angleDeg) {
        xCoordinate = x;
        yCoordinate = y;
        angleDegrees = angleDeg;
    }

    public FieldCoordinate(FieldCoordinate orig) {
        xCoordinate = orig.xCoordinate;
        yCoordinate = orig.yCoordinate;
        angleDegrees = orig.angleDegrees;

    }
    public double getX() { return xCoordinate; }
    public double getY() { return yCoordinate; }
    public double getAngleDegrees() { return angleDegrees; }
    public double getAngleRadians() { return Math.toRadians(angleDegrees); }
    public void setX(double x) { xCoordinate = x; }
    public void setY(double y) { yCoordinate = y; }
    public void setAngleDegrees(double angleDeg) { angleDegrees = angleDeg; }
    public void setAngleRadians(double angleRad) { angleDegrees = Math.toDegrees(angleRad); }
    public void setLocation(double x, double y, double angleDeg) {
        xCoordinate = x;
        yCoordinate = y;
        angleDegrees = angleDeg;
    }
}
