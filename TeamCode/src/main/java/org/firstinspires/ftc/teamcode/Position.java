package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 */
public class Position {

    // A rotation, in radians, in the interval (-pi, pi]
    private double rotation;

    protected String name;
    protected double x;
    protected double y;
    protected Navigation.Action action;
    //Add constructors if needed
    protected double strafePower = 0.0;
    protected double rotatePower = 0.0;

    Position() {
        x = 0.0;
        y = 0.0;
        name = "";
        setRotation(0.0);
    }

    Position(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        setRotation(theta);
    }

    Position(double x, double y, double theta, String name) {
        this.x = x;
        this.y = y;
        this.name = name;
        setRotation(theta);
    }

    Position(double x, double y, String name, Navigation.Action action, double strafePower, double rotatePower, double theta) {
        this.x = x;
        this.y = y;
        this.name = name;
        setRotation(theta);
        this.strafePower = strafePower;
        this.rotatePower = rotatePower;
        this.action = action;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public Position setRotation(double r) {
        this.rotation = r;
        return this;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getRotation() {
        return rotation;
    }

    public double getStrafePower() {
        return strafePower;
    }

    public double getRotatePower(){
        return rotatePower;
    }

    public Navigation.Action getAction(){
        return action;
    }

    public String getName(){
        return name;
    }

    public void reset() {
        setX(0.0);
        setY(0.0);
        setRotation(0.0);
    }

    public static Position add(Position a, Position b) {
        return new Position(a.getX() + b.getX(), a.getY() + b.getY(), (a.getRotation() + b.getRotation()) % (2 * Math.PI));
    }
}
