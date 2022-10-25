package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 */
public class Position {

    private Point location;

    // A rotation, in radians, in the interval (-pi, pi]
    private double rotation;

    Position() {
        location = new Point(0.0, 0.0, "");
        setRotation(0.0);
    }

    Position(double x, double y, double theta) {
        location = new Point(x, y, "");
        setRotation(theta);
    }
    Position(Point p, double r) {
        location = p;
        setRotation(r);
    }

    public void setX(double x) {
        location.setX(x);
    }

    public void setY(double y) {
        location.setY(y);
    }

    public Position setRotation(double r) {
        this.rotation = r;
        return this;
    }

    public double getX() {
        return location.x;
    }

    public double getY() {
        return location.y;
    }

    public double getRotation() {
        return rotation;
    }

    public Point getLocation() {return location;}

    public void reset() {
        setX(0.0);
        setY(0.0);
        setRotation(0.0);
    }

    public static Position add(Position a, Position b) {
        return new Position(a.getX() + b.getX(), a.getY() + b.getY(), (a.getRotation() + b.getRotation()) % (2 * Math.PI));
    }
}
