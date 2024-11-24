package com.kalipsorobotics.math;

public class Vector {

    final private double x;
    final private double y;

    public Vector (double x, double y) {
        this.x = x;
        this.y = y;
    }


//    public static Vector between(Point start, Point finish) {
//        double x = finish.getX() - start.getX();
//        double y = finish.getY() - start.getY();
//
//        return new Vector(x, y);
//    }

    public static Vector between(Position start, Position finish) {
        double x = finish.getX() - start.getX();
        double y = finish.getY() - start.getY();

        return new Vector(x, y);
    }

    public Vector scale(double factor) {
        return new Vector(
                this.getX() * factor,
                this.getY() * factor
        );
    }

    public Vector normalize(){
        return this.scale(1 / this.getLength());
    }

    public Vector withLength(double newLength) {
        return this.scale(newLength / this.getLength());
    }

//    public double dot(Point other) {
//        return (this.getX() * other.getX()) + (this.getY() * other.getY());
//    }

    public double dot(Position other) {
        return (this.getX() * other.getX()) + (this.getY() * other.getY());
    }

    public double dot(Vector other) {
        return (this.getX() * other.getX()) + (this.getY() * other.getY());
    }

    public double getHeadingDirection() {
        double theta = Math.atan2(this.getY(), this.getX());
        return theta;
    }

    public double getLength() {
        return Math.hypot(x, y);
    }


    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector add(Vector other) {
        return new Vector(this.getX() + other.getX(), this.getY() + other.getY());
    }

    public Velocity toVelocity(double theta) {
        return new Velocity(this.x, this.y, theta);
    }
}
