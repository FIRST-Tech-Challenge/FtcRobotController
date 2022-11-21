package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a orientation.
 */
public class Position {

    private Point location;

    // A orientation, in radians, in the interval (-pi, pi]
    private double orientation;

    Position() {
        location = new Point(0.0, 0.0);
        setOrientation(0.0);
    }

    Position(double x, double y, double theta) {
        location = new Point(x, y);
        setOrientation(theta);
    }
    Position(Point p, double r) {
        location = p;
        setOrientation(r);
    }

    public void setX(double x) {
        location.setX(x);
    }

    public void setY(double y) {
        location.setY(y);
    }

    public Position setOrientation(double r) {
        orientation = r;
        return this;
    }

    public double getX() {
        return location.x;
    }

    public double getY() {
        return location.y;
    }

    public double getOrientation() {
        return orientation;
    }

    public Point getLocation() {return location;}

    public void reset() {
        setX(0.0);
        setY(0.0);
        setOrientation(0.0);
    }

    public static Position add(Position a, Position b) {
        return new Position(a.getX() + b.getX(), a.getY() + b.getY(), (a.getOrientation() + b.getOrientation()) % (2 * Math.PI));
    }
}
