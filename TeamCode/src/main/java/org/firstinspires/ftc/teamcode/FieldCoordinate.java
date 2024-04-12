package org.firstinspires.ftc.teamcode;

public class FieldCoordinate {
    protected double xCoordinate;
    protected double yCoordinate;
    protected double angleRadians;
    public FieldCoordinate(double x, double y, double angleRad) {
        xCoordinate = x;
        yCoordinate = y;
        angleRadians = angleRad;
    }

    public double getX() { return xCoordinate; }
    public double getY() { return yCoordinate; }
    public double getAngleRadians() { return angleRadians; }
    public void setX(double x) { xCoordinate = x; }
    public void setY(double y) { yCoordinate = y; }
    public void setAngleRadians(double angleRad) { angleRadians = angleRad; }
    public void setLocation(double x, double y, double angleRad) {
        xCoordinate = x;
        yCoordinate = y;
        angleRadians = angleRad;
    }
}
