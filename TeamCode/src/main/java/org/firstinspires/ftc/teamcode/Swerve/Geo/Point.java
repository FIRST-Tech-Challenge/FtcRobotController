package org.firstinspires.ftc.teamcode.Swerve.Geo;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Point {
    public double x, y;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point(){
        this(0,0);
    }
    public double getX() {
        return this.x;
    }
    public double getY() {
        return this.y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public Point subtract(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public Point add(Point other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point add(double scalar) {
        return new Point(x + scalar, y + scalar);
    }

    Point subt(Point other) {
        return new Point(getX() - other.getX(), getY() - other.getY());
    }

    public double distanceTo(Point other) {
        return other.subtract(this).radius();
    }

    public double atan(){
        return Math.atan2(x, y);
    }

    public double radius() {
        return Math.hypot(x, y);
    }

    public static Point polar(double r, double a){
        return new Point(Math.cos(a)*r, Math.sin(a)*r);
    }

    public Point rotate(double amount){
        double X = x * cos(amount) - y * sin(amount);
        double Y = x * sin(amount) + y * cos(amount);
        return new Point(X, Y);
    }

}
