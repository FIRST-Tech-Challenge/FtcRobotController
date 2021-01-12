package org.firstinspires.ftc.teamcode.drivecontrol;

public class Pos {
    private double x;
    private double y;
    private double heading;

    public Pos(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() { return heading; }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double h) { this.heading = h; }

    public void incrementX(double i) { this.x += i; }

    public void incrementY(double i) { this.y += i; }

    public void incrementHeading(double h) { this.heading += h; }
}
