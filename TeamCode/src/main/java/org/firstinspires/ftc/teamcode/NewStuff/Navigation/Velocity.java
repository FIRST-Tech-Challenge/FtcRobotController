package org.firstinspires.ftc.teamcode.NewStuff.Navigation;

class Velocity {
    final private double x;
    final private double y;
    final private double theta;

    Velocity(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
//add vector to vector
    public Velocity add (Velocity velocity) {
        return new Velocity(
          this.x + velocity.x,
          this.y + velocity.y,
          this.theta + velocity.y
        );
    }

    @Override
    public String toString() {
        return "{" +
                "x=" + x +
                ", y=" + y +
                ", theta=" + theta +
                '}';
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }
}