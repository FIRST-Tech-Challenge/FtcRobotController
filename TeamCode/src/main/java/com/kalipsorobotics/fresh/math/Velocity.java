package com.kalipsorobotics.fresh.math;

public class Velocity {
    final private double x;
    final private double y;
    final private double theta;

    public Velocity(double x, double y, double theta) {
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

    public Velocity divide(double denominator) {
        return new Velocity(this.x / denominator, this.y / denominator, this.theta / denominator);
    }
    public boolean isWithinThreshhold(double thresholdX, double threshholdY, double threshholdTheta) {
        return (Math.abs(this.x) < thresholdX && Math.abs(this.y) < threshholdY && Math.abs(this.theta) < threshholdTheta);
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