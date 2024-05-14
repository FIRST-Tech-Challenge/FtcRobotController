package org.firstinspires.ftc.teamcode;

public class FieldCoordinate {
    protected double xCoordinate;
    protected double yCoordinate;
    protected double angleDeg;
    public FieldCoordinate(double x, double y, double angleDeg) {
        xCoordinate = x;
        yCoordinate = y;
        this.angleDeg = angleDeg;
    }

    public FieldCoordinate(FieldCoordinate orig) {
        xCoordinate = orig.xCoordinate;
        yCoordinate = orig.yCoordinate;
        angleDeg = orig.angleDeg;

    }
    public double getX() { return xCoordinate; }
    public double getY() { return yCoordinate; }
    public double getAngleDegrees() { return angleDeg; }
    public double getAngleRadians() { return Math.toRadians(angleDeg); }
    public void setX(double x) { xCoordinate = x; }
    public void setY(double y) { yCoordinate = y; }
    public void setAngleDegrees(double angleDeg) { this.angleDeg = angleDeg; }
    public void setAngleRadians(double angleRad) { angleDeg = Math.toDegrees(angleRad); }
    public void setLocation(double x, double y, double angleRad) {
        xCoordinate = x;
        yCoordinate = y;
        angleDeg = angleDeg;
    }
}
